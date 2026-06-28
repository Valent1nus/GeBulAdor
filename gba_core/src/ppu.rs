//! La **PPU** (Picture Processing Unit): el subsistema gráfico de la GBA.
//! Mini-Hitos **2.4a–2.4c** (bitmap modo 3 → barrido por scanlines → fondos de tiles).
//!
//! ## De "frame de una vez" a "línea a línea"
//!
//! El Mini-Hito 2.4a componía el frame **completo** bajo demanda (un único volcado
//! de la VRAM al framebuffer). El hardware real **no** funciona así: la PPU dibuja
//! la pantalla **línea a línea** (*scanline*), y entre línea y línea hay huecos de
//! tiempo —el **H-Blank**— y, tras las 160 líneas visibles, un hueco largo —el
//! **V-Blank**— de 68 líneas más. Los juegos aprovechan esos huecos para tocar los
//! registros gráficos *en mitad del frame* (efectos de agua, degradados, *raster
//! effects*): si se renderizara el frame entero al final, esos cambios por línea se
//! perderían. Por eso este hito pasa a un modelo dirigido por el [`crate::Scheduler`]:
//!
//! - Cada línea dura **1232 ciclos** (240 puntos visibles × 4 ciclos = 960 de
//!   *H-Draw*, + 272 de *H-Blank*). Ver [`SCANLINE_CYCLES`].
//! - Hay **228 líneas** por frame ([`TOTAL_SCANLINES`]): 160 visibles (0–159) y 68
//!   de V-Blank (160–227). Un frame completo son [`CYCLES_PER_FRAME`] ciclos.
//! - El bus programa en el scheduler dos eventos por línea —entrar en H-Blank y
//!   terminar la línea— y, al dispararse, esta PPU **renderiza esa scanline** y
//!   actualiza sus *flags* e interrupciones (ver [`crate::Bus::sync_to_cycle`]).
//!
//! ## Los registros que gobiernan el barrido
//!
//! Esta PPU es la **fuente de verdad** de tres registros (el bus enruta aquí sus
//! accesos, igual que con `DISPCNT` en 2.4a):
//!
//! - **`DISPCNT`** (`0x0400_0000`): control de pantalla. De él se usan el **modo de
//!   vídeo** (bits 0-2) y el ***forced blank*** (bit 7), como en 2.4a.
//! - **`DISPSTAT`** (`0x0400_0004`): estado y control del barrido. Sus bits bajos
//!   (0-2) son ***flags* de solo lectura** que la PPU pone/quita según dónde esté el
//!   barrido (V-Blank, H-Blank, coincidencia de V-Counter); los bits 3-5 son los
//!   **enables de IRQ** de esas tres fuentes (los escribe el juego); y los bits 8-15
//!   son **`LYC`**, la línea con la que comparar para la IRQ de V-Counter.
//! - **`VCOUNT`** (`0x0400_0006`, solo lectura): la **línea actual** (0–227).
//!
//! ## Las tres interrupciones de la pantalla
//!
//! Al avanzar el barrido, la PPU solicita (vía [`crate::InterruptControl`], como los
//! timers en 2.3e) hasta tres IRQs, cada una si su *enable* de `DISPSTAT` está a 1:
//! **V-Blank** (al entrar en la línea 160), **H-Blank** (al entrar en el H-Blank de
//! *cada* línea) y **V-Counter** (cuando la nueva línea coincide con `LYC`). Es lo
//! que **destraba** el `VBlankIntrWait` (Mini-Hito 2.3a-bis) y el arnés `r12` de las
//! gba-tests, que esperan en bucle a que el *flag* de V-Blank cambie.
//!
//! ## El framebuffer vive aquí (cambio de diseño respecto a 2.4a)
//!
//! Como ahora la imagen se compone **durante la ejecución** (en los eventos del
//! scheduler, que el bus dispara sin acceso al framebuffer del [`crate::Gba`]), la
//! PPU **posee** su propio framebuffer RGBA y va escribiendo en él scanline a
//! scanline. El bus le **presta** la VRAM y la PRAM al renderizar (siguen viviendo
//! en él), y expone el resultado por [`crate::Bus::framebuffer`]; el `Gba` ya no
//! guarda el buffer, solo lo reenvía. La salida del núcleo —un buffer RGBA de
//! 240×160— no cambia.
//!
//! ## Qué queda para los siguientes hitos
//!
//! Tras 2.4c esta PPU dibuja los **fondos de texto** de los modos 0 y 1 (BG0–3 /
//! BG0–1) además del **modo 3** bitmap, todo por scanlines. Quedan: los fondos
//! **afines** (modo 2 entero y el BG2 del modo 1) para el 2.4f, los **sprites** (OAM)
//! para el 2.4d, los **modos bitmap 4/5** para el 2.4e y los efectos (ventanas,
//! blending, mosaico) para 2.4g–2.4h. El resto de bits de `DISPCNT`/`BGxCNT` no
//! usados aquí (p. ej. *mosaic*) se almacenan sin efecto.

use crate::interrupt::{Interrupt, InterruptControl};
use crate::{BYTES_PER_PIXEL, FRAMEBUFFER_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH};

// ---- Registros y sus offsets (dentro de la región de I/O, base 0x0400_0000) ----

/// `DISPCNT` (control de pantalla, 16 bits): bytes `0x000`–`0x001`.
const DISPCNT_LO: u32 = 0x000;
/// `DISPSTAT` (estado/control del barrido, 16 bits): bytes `0x004`–`0x005`.
const DISPSTAT_LO: u32 = 0x004;
/// `VCOUNT` (línea actual, 16 bits, solo lectura): bytes `0x006`–`0x007`.
const VCOUNT_LO: u32 = 0x006;
/// `BG0CNT` (control del fondo 0, 16 bits): bytes `0x008`–`0x009`. Los cuatro
/// `BGxCNT` ocupan `0x008`–`0x00F` (2 bytes cada uno).
const BG0CNT_LO: u32 = 0x008;
/// `BG0HOFS` (scroll horizontal del fondo 0, 16 bits, solo escritura): bytes
/// `0x010`–`0x011`. Los 8 registros `BGxHOFS`/`BGxVOFS` ocupan `0x010`–`0x01F`
/// intercalados (HOFS, VOFS, HOFS, VOFS… por fondo).
const BG0HOFS_LO: u32 = 0x010;
/// Primer byte **después** del bloque de registros de scroll de fondo (`0x020`).
const BG_REGS_END: u32 = 0x020;

/// Máscara del **modo de vídeo** en `DISPCNT` (bits 0-2).
const BG_MODE_MASK: u16 = 0b111;
/// Bit de ***forced blank*** en `DISPCNT` (bit 7): pantalla en blanco.
const FORCED_BLANK: u16 = 1 << 7;
/// Bit del *enable* del fondo 0 en `DISPCNT` (bit 8). Los fondos BG0–BG3 ocupan los
/// bits 8-11: el fondo `n` se muestra si su bit `8 + n` está a 1.
const DISPCNT_BG0_ENABLE: u16 = 1 << 8;

// Bits de `DISPSTAT`.
/// *Flag* de V-Blank (bit 0, solo lectura): 1 mientras el barrido está en V-Blank.
const DISPSTAT_VBLANK_FLAG: u16 = 1 << 0;
/// *Flag* de H-Blank (bit 1, solo lectura): 1 durante el H-Blank de cada línea.
const DISPSTAT_HBLANK_FLAG: u16 = 1 << 1;
/// *Flag* de coincidencia de V-Counter (bit 2, solo lectura): 1 si `VCOUNT == LYC`.
const DISPSTAT_VCOUNT_FLAG: u16 = 1 << 2;
/// *Enable* de la IRQ de V-Blank (bit 3, escribible).
const DISPSTAT_VBLANK_IRQ: u16 = 1 << 3;
/// *Enable* de la IRQ de H-Blank (bit 4, escribible).
const DISPSTAT_HBLANK_IRQ: u16 = 1 << 4;
/// *Enable* de la IRQ de V-Counter (bit 5, escribible).
const DISPSTAT_VCOUNT_IRQ: u16 = 1 << 5;
/// Bits **escribibles** del byte bajo de `DISPSTAT` (los tres enables, 3-5). Los
/// bits 0-2 son *flags* de solo lectura y los 6-7 no se usan.
const DISPSTAT_LOW_WRITABLE: u16 = DISPSTAT_VBLANK_IRQ | DISPSTAT_HBLANK_IRQ | DISPSTAT_VCOUNT_IRQ;

// ---- Campos de `BGxCNT` (control de un fondo de tiles, Mini-Hito 2.4c) ----

/// Máscara de la **prioridad** del fondo en `BGxCNT` (bits 0-1): 0 = más al frente.
const BGCNT_PRIORITY_MASK: u16 = 0b11;
/// Bit de **color de 256 colores / 8 bpp** en `BGxCNT` (bit 7): si está a 0, el
/// fondo usa 16 paletas de 16 colores (4 bpp).
const BGCNT_8BPP: u16 = 1 << 7;
/// Tamaño en bytes de un **bloque de caracteres** (*char base block*, bits 2-3 de
/// `BGxCNT` × esto): 16 KiB.
const CHAR_BLOCK_BYTES: usize = 0x4000;
/// Tamaño en bytes de un **bloque de mapa** (*screen base block*, bits 8-12 de
/// `BGxCNT` × esto): 2 KiB = 1024 entradas de mapa de 16 bits.
const SCREEN_BLOCK_BYTES: usize = 0x800;
/// Bytes de un tile en 4 bpp (8×8 píxeles, medio byte cada uno): 32.
const TILE_BYTES_4BPP: usize = 32;
/// Bytes de un tile en 8 bpp (8×8 píxeles, un byte cada uno): 64.
const TILE_BYTES_8BPP: usize = 64;

// Campos de una **entrada de mapa de tiles** (16 bits, modo texto).
/// Máscara del **número de tile** (bits 0-9).
const MAP_TILE_MASK: u16 = 0x3FF;
/// Bit de **volteo horizontal** del tile (bit 10).
const MAP_HFLIP: u16 = 1 << 10;
/// Bit de **volteo vertical** del tile (bit 11).
const MAP_VFLIP: u16 = 1 << 11;

// ---- Temporización del barrido (GBATEK: 1 punto = 4 ciclos de CPU) ----

/// Ciclos de CPU por punto (píxel) dibujado.
const CYCLES_PER_DOT: u64 = 4;
/// Ciclos de la parte **visible** de una línea (*H-Draw*): 240 puntos × 4 = 960.
pub const HDRAW_CYCLES: u64 = SCREEN_WIDTH as u64 * CYCLES_PER_DOT;
/// Ciclos del **H-Blank** de una línea: 68 puntos × 4 = 272.
pub const HBLANK_CYCLES: u64 = 68 * CYCLES_PER_DOT;
/// Ciclos totales de una línea (visible + H-Blank): 960 + 272 = 1232.
pub const SCANLINE_CYCLES: u64 = HDRAW_CYCLES + HBLANK_CYCLES;
/// Número total de líneas por frame: 160 visibles + 68 de V-Blank = 228.
pub const TOTAL_SCANLINES: u16 = 228;
/// Ciclos de un frame completo (refresco): 228 × 1232 = 280 896.
pub const CYCLES_PER_FRAME: u64 = SCANLINE_CYCLES * TOTAL_SCANLINES as u64;

/// Primera línea de V-Blank (las visibles son 0..[`SCREEN_HEIGHT`]).
const FIRST_VBLANK_LINE: u16 = SCREEN_HEIGHT as u16;

/// Color RGBA del *forced blank*: blanco opaco.
const WHITE: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];

/// La unidad de proceso gráfico. Vive dentro del [`crate::Bus`].
pub struct Ppu {
    /// Registro de control de pantalla `DISPCNT` (`0x0400_0000`).
    dispcnt: u16,
    /// Registro de estado/control del barrido `DISPSTAT` (`0x0400_0004`).
    dispstat: u16,
    /// Línea actual del barrido (`VCOUNT`, 0–227). La avanza el evento de fin de
    /// línea ([`Ppu::enter_next_line`]); el resto del hardware la lee.
    vcount: u16,
    /// Control de cada fondo `BG0CNT`–`BG3CNT` (`0x0400_0008`–`0x0400_000F`,
    /// Mini-Hito 2.4c): prioridad, bloque de *tiles*, *mosaic*, 4/8 bpp, bloque de
    /// mapa y tamaño. Ver [`Ppu::render_scanline`].
    bgcnt: [u16; 4],
    /// *Scroll* horizontal de cada fondo `BG0HOFS`–`BG3HOFS` (`0x0400_0010`+, de
    /// **solo escritura**, 9 bits útiles).
    bghofs: [u16; 4],
    /// *Scroll* vertical de cada fondo `BG0VOFS`–`BG3VOFS` (`0x0400_0012`+, de
    /// **solo escritura**, 9 bits útiles).
    bgvofs: [u16; 4],
    /// Framebuffer RGBA del núcleo ([`FRAMEBUFFER_SIZE`] bytes). La PPU lo va
    /// rellenando **scanline a scanline** ([`Ppu::render_scanline`]); el frontend lo
    /// lee a través de [`crate::Bus::framebuffer`].
    framebuffer: Vec<u8>,
}

impl Ppu {
    /// Crea la PPU en su estado de reset: todos los registros a 0 (modo 0, barrido en
    /// la línea 0, sin *flags* ni IRQs) y el framebuffer a negro.
    pub fn new() -> Self {
        Ppu {
            dispcnt: 0,
            dispstat: 0,
            vcount: 0,
            bgcnt: [0; 4],
            bghofs: [0; 4],
            bgvofs: [0; 4],
            framebuffer: vec![0; FRAMEBUFFER_SIZE],
        }
    }

    /// `true` si el offset de I/O `io_off` cae en un registro que gestiona la PPU:
    /// `DISPCNT`, `DISPSTAT`, `VCOUNT` y, desde 2.4c, los `BGxCNT`/`BGxHOFS`/`BGxVOFS`
    /// (`0x008`–`0x01F`). Lo usa el bus para enrutar aquí el acceso. El hueco
    /// `0x002`–`0x003` (*green swap*, no implementado) queda fuera.
    pub fn handles(io_off: u32) -> bool {
        (DISPCNT_LO..DISPCNT_LO + 2).contains(&io_off)
            || (DISPSTAT_LO..BG_REGS_END).contains(&io_off)
    }

    /// Lee un byte de un registro de la PPU. Nunca panica: un offset fuera de los
    /// registros modelados devuelve 0.
    pub fn read_u8(&self, io_off: u32) -> u8 {
        match io_off {
            DISPCNT_LO => self.dispcnt as u8,
            n if n == DISPCNT_LO + 1 => (self.dispcnt >> 8) as u8,
            DISPSTAT_LO => self.dispstat as u8,
            n if n == DISPSTAT_LO + 1 => (self.dispstat >> 8) as u8,
            VCOUNT_LO => self.vcount as u8,
            n if n == VCOUNT_LO + 1 => (self.vcount >> 8) as u8,
            // BGxCNT (0x008–0x00F): legible. idx = qué fondo, bit bajo = qué byte.
            n if (BG0CNT_LO..BG0HOFS_LO).contains(&n) => {
                let idx = ((n - BG0CNT_LO) / 2) as usize;
                (self.bgcnt[idx] >> (((n & 1) * 8) as u16)) as u8
            }
            // BGxHOFS/BGxVOFS (0x010–0x01F): de solo escritura, se leen como 0.
            _ => 0,
        }
    }

    /// Escribe un byte en un registro de la PPU, respetando qué es escribible:
    /// `DISPCNT` entero; de `DISPSTAT`, solo los *enables* de IRQ (bits 3-5) y `LYC`
    /// (bits 8-15) —los *flags* (0-2) son de solo lectura, los pone la PPU—; y
    /// `VCOUNT` es de **solo lectura** (la escritura se ignora). Nunca panica.
    pub fn write_u8(&mut self, io_off: u32, value: u8) {
        match io_off {
            DISPCNT_LO => self.dispcnt = (self.dispcnt & 0xFF00) | u16::from(value),
            n if n == DISPCNT_LO + 1 => {
                self.dispcnt = (self.dispcnt & 0x00FF) | (u16::from(value) << 8)
            }
            // Byte bajo de DISPSTAT: solo los enables (bits 3-5) son escribibles;
            // se conservan los flags (0-2) y el byte alto (LYC).
            DISPSTAT_LO => {
                let writable = u16::from(value) & DISPSTAT_LOW_WRITABLE;
                self.dispstat = (self.dispstat & !0x00FF) | (self.dispstat & 0x0007) | writable;
            }
            // Byte alto de DISPSTAT: LYC (bits 8-15). Al cambiarlo, reevaluamos el
            // flag de coincidencia (sin disparar IRQ: eso solo pasa al cambiar de
            // línea, ver enter_next_line).
            n if n == DISPSTAT_LO + 1 => {
                self.dispstat = (self.dispstat & 0x00FF) | (u16::from(value) << 8);
                let matches = self.vcount == self.lyc();
                self.set_flag(DISPSTAT_VCOUNT_FLAG, matches);
            }
            // BGxCNT (0x008–0x00F): control de fondo, escribible entero.
            n if (BG0CNT_LO..BG0HOFS_LO).contains(&n) => {
                let idx = ((n - BG0CNT_LO) / 2) as usize;
                write_byte(&mut self.bgcnt[idx], n & 1, value);
            }
            // BGxHOFS/BGxVOFS (0x010–0x01F): scroll, intercalados HOFS, VOFS por
            // fondo (4 bytes por fondo). El bit 2 del offset distingue VOFS de HOFS.
            n if (BG0HOFS_LO..BG_REGS_END).contains(&n) => {
                let idx = ((n - BG0HOFS_LO) / 4) as usize;
                let reg = if n & 0b10 == 0 {
                    &mut self.bghofs[idx]
                } else {
                    &mut self.bgvofs[idx]
                };
                write_byte(reg, n & 1, value);
            }
            // VCOUNT es de solo lectura.
            _ => {}
        }
    }

    /// El **modo de vídeo** activo (0–7), de los bits 0-2 de `DISPCNT`. De los
    /// válidos, este hito solo dibuja el 3 (los demás pintan el *backdrop*).
    pub fn mode(&self) -> u8 {
        (self.dispcnt & BG_MODE_MASK) as u8
    }

    /// `true` si el ***forced blank*** (bit 7 de `DISPCNT`) está activo: la pantalla
    /// se muestra en blanco.
    pub fn forced_blank(&self) -> bool {
        self.dispcnt & FORCED_BLANK != 0
    }

    /// La línea actual del barrido (`VCOUNT`, 0–227).
    pub fn vcount(&self) -> u16 {
        self.vcount
    }

    /// El framebuffer RGBA ya compuesto (la única salida visual del núcleo).
    pub fn framebuffer(&self) -> &[u8] {
        &self.framebuffer
    }

    // ---- Avance del barrido (lo dispara el scheduler vía el bus) -------------

    /// Entra en el **H-Blank** de la línea actual: pone el *flag* de H-Blank y, si su
    /// IRQ está habilitada, la solicita. La llama [`crate::Bus::sync_to_cycle`] al
    /// vencer el evento de H-Blank (a 960 ciclos del inicio de la línea), tras lo
    /// cual el bus renderiza esta scanline si es visible.
    pub fn enter_hblank(&mut self, irq: &mut InterruptControl) {
        self.dispstat |= DISPSTAT_HBLANK_FLAG;
        if self.dispstat & DISPSTAT_HBLANK_IRQ != 0 {
            irq.request(Interrupt::HBlank);
        }
    }

    /// Termina la línea actual y **avanza a la siguiente** (al cabo de los 1232
    /// ciclos): limpia el *flag* de H-Blank, incrementa `VCOUNT` (con vuelta a 0 tras
    /// la última línea) y actualiza los *flags* de V-Blank y V-Counter, solicitando
    /// las IRQs de V-Blank (al entrar en la línea 160), V-Counter (si la nueva línea
    /// coincide con `LYC`) y, así, dejando el barrido listo para la siguiente.
    ///
    /// Devuelve `true` si con este avance se acaba de **entrar en V-Blank** (la nueva
    /// línea es la 160), lo que el bus usa para disparar el DMA de V-Blank.
    pub fn enter_next_line(&mut self, irq: &mut InterruptControl) -> bool {
        // La nueva línea empieza por su parte visible: el H-Blank ya no aplica.
        self.dispstat &= !DISPSTAT_HBLANK_FLAG;

        self.vcount += 1;
        if self.vcount >= TOTAL_SCANLINES {
            self.vcount = 0;
        }
        let line = self.vcount;

        // Flag de V-Blank: activo en las líneas 160..=226 (la 227 ya está "saliendo"
        // del V-Blank, según GBATEK, y la tiene a 0).
        let in_vblank = (FIRST_VBLANK_LINE..TOTAL_SCANLINES - 1).contains(&line);
        self.set_flag(DISPSTAT_VBLANK_FLAG, in_vblank);

        // Flag + IRQ de coincidencia de V-Counter (VCOUNT == LYC).
        let vcount_match = line == self.lyc();
        self.set_flag(DISPSTAT_VCOUNT_FLAG, vcount_match);
        if vcount_match && self.dispstat & DISPSTAT_VCOUNT_IRQ != 0 {
            irq.request(Interrupt::VCounter);
        }

        // IRQ de V-Blank: solo en el flanco de entrada (al llegar a la línea 160).
        let entered_vblank = line == FIRST_VBLANK_LINE;
        if entered_vblank && self.dispstat & DISPSTAT_VBLANK_IRQ != 0 {
            irq.request(Interrupt::VBlank);
        }
        entered_vblank
    }

    /// `true` si alguna IRQ de la pantalla podría **despertar** a la CPU de un `Halt`:
    /// su *enable* está puesto en `DISPSTAT` **y** su fuente habilitada en `IE`. Es el
    /// equivalente a [`crate::Timers::can_wake`] para que [`crate::Bus::next_wakeup_cycle`]
    /// sepa que adelantar el reloj hasta el próximo evento de la PPU puede generar la
    /// IRQ que termina el `Halt` (lo necesita `VBlankIntrWait`).
    pub fn can_wake(&self, irq: &InterruptControl) -> bool {
        let armed = |enable: u16, source: Interrupt| {
            self.dispstat & enable != 0 && irq.is_enabled(source)
        };
        armed(DISPSTAT_VBLANK_IRQ, Interrupt::VBlank)
            || armed(DISPSTAT_HBLANK_IRQ, Interrupt::HBlank)
            || armed(DISPSTAT_VCOUNT_IRQ, Interrupt::VCounter)
    }

    // ---- Renderizado --------------------------------------------------------

    /// Renderiza **una scanline** (`y`, 0–159) en el framebuffer propio, leyendo la
    /// `vram`/`pram` que le presta el bus. Una `y` no visible (≥ [`SCREEN_HEIGHT`]) no
    /// pinta nada (las líneas de V-Blank no tienen píxeles).
    ///
    /// El orden de decisión reproduce el del hardware: *forced blank* → línea blanca;
    /// modos de **tiles** 0/1 (2.4c) → composición de fondos por prioridad; modo 3 →
    /// bitmap directo 16bpp de la VRAM; cualquier otro modo (aún sin implementar) → el
    /// *backdrop* (`PRAM[0]`).
    ///
    /// > Los fondos **afines** (modo 2 entero, y el BG2 del modo 1) son del Mini-Hito
    /// > 2.4f; aquí solo se dibujan los fondos de **texto** (modo 0: BG0–3; modo 1:
    /// > BG0–1). Los sprites llegan en 2.4d.
    pub fn render_scanline(&mut self, y: u16, vram: &[u8], pram: &[u8]) {
        let y = y as usize;
        if y >= SCREEN_HEIGHT {
            return;
        }
        // Copiamos el estado de registros que necesita el render a locales antes de
        // tomar prestado el framebuffer mutablemente (evita un conflicto de préstamos
        // con `self`): todos son `Copy`.
        let dispcnt = self.dispcnt;
        let bgcnt = self.bgcnt;
        let bghofs = self.bghofs;
        let bgvofs = self.bgvofs;

        let start = y * SCREEN_WIDTH * BYTES_PER_PIXEL;
        let row = &mut self.framebuffer[start..start + SCREEN_WIDTH * BYTES_PER_PIXEL];

        if dispcnt & FORCED_BLANK != 0 {
            fill_row(row, WHITE);
            return;
        }
        match (dispcnt & BG_MODE_MASK) as u8 {
            mode @ (0 | 1) => render_tiled_row(row, y, mode, dispcnt, &bgcnt, &bghofs, &bgvofs, vram, pram),
            3 => {
                // Cada píxel: 2 bytes BGR555 desde la VRAM, fila a fila.
                let row_base = y * SCREEN_WIDTH * 2;
                for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
                    let off = row_base + x * 2;
                    let color = u16::from_le_bytes([
                        vram.get(off).copied().unwrap_or(0),
                        vram.get(off + 1).copied().unwrap_or(0),
                    ]);
                    out.copy_from_slice(&bgr555_to_rgba(color));
                }
            }
            _ => fill_row(row, backdrop(pram)),
        }
    }

    /// Renderiza el **frame completo** de una vez (las 160 líneas visibles), como en
    /// 2.4a. Es el render **bajo demanda** que conservan el frontend para un volcado
    /// inmediato y los tests; el render por scanlines del bucle usa
    /// [`Ppu::render_scanline`] línea a línea.
    pub fn render_frame(&mut self, vram: &[u8], pram: &[u8]) {
        for y in 0..SCREEN_HEIGHT as u16 {
            self.render_scanline(y, vram, pram);
        }
    }

    /// Rellena **todo** el framebuffer con un color RGBA sólido (lo usa
    /// [`crate::Bus::clear_framebuffer`] / [`crate::Gba::clear`]).
    pub fn clear_framebuffer(&mut self, rgba: [u8; 4]) {
        for out in self.framebuffer.chunks_exact_mut(BYTES_PER_PIXEL) {
            out.copy_from_slice(&rgba);
        }
    }

    // ---- Internos -----------------------------------------------------------

    /// El valor `LYC` (línea con la que comparar para la IRQ de V-Counter), en los
    /// bits 8-15 de `DISPSTAT`.
    fn lyc(&self) -> u16 {
        self.dispstat >> 8
    }

    /// Pone (`on`) o quita un *flag* de `DISPSTAT` (una máscara de un bit).
    fn set_flag(&mut self, mask: u16, on: bool) {
        if on {
            self.dispstat |= mask;
        } else {
            self.dispstat &= !mask;
        }
    }
}

impl Default for Ppu {
    fn default() -> Self {
        Self::new()
    }
}

/// Convierte un color **BGR555** (15 bits, el formato nativo de la GBA) a
/// **RGBA8888** (el del framebuffer del núcleo).
///
/// El empaquetado BGR555 es `0bX_BBBBB_GGGGG_RRRRR`: 5 bits por canal, rojo en los
/// bits bajos, y el bit 15 sin usar. Para escalar cada canal de 5 a 8 bits se
/// replican los bits altos en los bajos (`c8 = (c5 << 3) | (c5 >> 2)`), que reparte
/// los 32 niveles de forma uniforme por el rango 0–255 (0→0, 31→255), en vez de
/// dejar oscuros los máximos como haría un simple desplazamiento. El alfa es siempre
/// `0xFF` (opaco), como el resto del framebuffer.
pub fn bgr555_to_rgba(color: u16) -> [u8; 4] {
    let r5 = (color & 0x1F) as u8;
    let g5 = ((color >> 5) & 0x1F) as u8;
    let b5 = ((color >> 10) & 0x1F) as u8;
    [expand5(r5), expand5(g5), expand5(b5), 0xFF]
}

/// Escala un canal de color de 5 bits (0–31) a 8 bits (0–255) replicando los bits
/// altos en los bajos. Ver [`bgr555_to_rgba`].
#[inline]
fn expand5(c5: u8) -> u8 {
    (c5 << 3) | (c5 >> 2)
}

/// El color de fondo (*backdrop*): la entrada 0 de la paleta, en `PRAM[0..2]`
/// (BGR555 *little-endian*), convertida a RGBA. Lee con `get` para no panicar nunca.
fn backdrop(pram: &[u8]) -> [u8; 4] {
    let color = u16::from_le_bytes([
        pram.first().copied().unwrap_or(0),
        pram.get(1).copied().unwrap_or(0),
    ]);
    bgr555_to_rgba(color)
}

/// Rellena una fila del framebuffer (`row`, ya recortada a una scanline) con un
/// mismo color RGBA.
fn fill_row(row: &mut [u8], rgba: [u8; 4]) {
    for out in row.chunks_exact_mut(BYTES_PER_PIXEL) {
        out.copy_from_slice(&rgba);
    }
}

/// Lee un byte de un buffer (VRAM/PRAM) sin panicar: un offset fuera de rango
/// devuelve 0 (defensa: la entrada de mapa la controla la ROM y podría apuntar lejos).
#[inline]
fn read_at(buf: &[u8], off: usize) -> u8 {
    buf.get(off).copied().unwrap_or(0)
}

/// Escribe el byte bajo (`byte_sel == 0`) o alto (`!= 0`) de un registro de 16 bits,
/// conservando el otro. Lo usan los registros de fondo de [`Ppu::write_u8`].
fn write_byte(reg: &mut u16, byte_sel: u32, value: u8) {
    if byte_sel == 0 {
        *reg = (*reg & 0xFF00) | u16::from(value);
    } else {
        *reg = (*reg & 0x00FF) | (u16::from(value) << 8);
    }
}

/// Renderiza una scanline `y` en los **modos de tiles** 0 y 1 (Mini-Hito 2.4c),
/// componiendo los fondos de **texto** activos por orden de prioridad sobre el
/// *backdrop*.
///
/// Solo se dibujan los fondos de texto válidos en el modo (modo 0 → BG0–3; modo 1 →
/// BG0–1; el BG2 afín del modo 1 es del 2.4f) **y** habilitados en `DISPCNT`. Se
/// ordenan por (prioridad, índice de fondo): para cada píxel se toma el primer fondo
/// —el más al frente— que aporte un color no transparente (índice de paleta ≠ 0); si
/// ninguno lo hace, queda el *backdrop*.
#[allow(clippy::too_many_arguments)]
fn render_tiled_row(
    row: &mut [u8],
    y: usize,
    mode: u8,
    dispcnt: u16,
    bgcnt: &[u16; 4],
    bghofs: &[u16; 4],
    bgvofs: &[u16; 4],
    vram: &[u8],
    pram: &[u8],
) {
    // Fondos de texto que el modo permite dibujar (los afines son del 2.4f).
    let text_bgs: &[usize] = match mode {
        0 => &[0, 1, 2, 3],
        1 => &[0, 1],
        _ => &[],
    };
    // De ellos, los habilitados en DISPCNT (bits 8-11), ordenados por prioridad.
    let mut order = [0usize; 4];
    let mut count = 0;
    for &bg in text_bgs {
        if dispcnt & (DISPCNT_BG0_ENABLE << bg) != 0 {
            order[count] = bg;
            count += 1;
        }
    }
    let order = &mut order[..count];
    // Estable y por (prioridad, índice): a igual prioridad, el fondo de menor índice
    // queda delante (regla de la GBA).
    order.sort_by_key(|&bg| ((bgcnt[bg] & BGCNT_PRIORITY_MASK) as u8, bg as u8));

    let backdrop = backdrop(pram);
    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        let mut rgba = backdrop;
        for &bg in order.iter() {
            if let Some(color) = sample_text_bg(bgcnt[bg], bghofs[bg], bgvofs[bg], x, y, vram, pram) {
                rgba = bgr555_to_rgba(color);
                break;
            }
        }
        out.copy_from_slice(&rgba);
    }
}

/// Muestrea el color de un **fondo de texto** en el píxel de pantalla `(screen_x,
/// screen_y)`, aplicando su *scroll*. Devuelve el color **BGR555** del píxel, o `None`
/// si es transparente (índice de paleta 0). Nunca panica: todo acceso a `vram`/`pram`
/// va por `get`.
///
/// Reproduce el pipeline del hardware: *scroll* → envoltura al tamaño del fondo →
/// localizar la entrada de mapa en su *screen base block* → leer tile, volteos y
/// paleta → resolver el píxel del tile en 4 u 8 bpp desde el *char base block*.
fn sample_text_bg(
    bgcnt: u16,
    hofs: u16,
    vofs: u16,
    screen_x: usize,
    screen_y: usize,
    vram: &[u8],
    pram: &[u8],
) -> Option<u16> {
    let (width_tiles, height_tiles) = bg_size_tiles(bgcnt);
    // Coordenada dentro del fondo, envuelta a su tamaño total (potencia de dos).
    let bg_x = (screen_x + hofs as usize) & (width_tiles * 8 - 1);
    let bg_y = (screen_y + vofs as usize) & (height_tiles * 8 - 1);
    let (tile_x, tile_y) = (bg_x / 8, bg_y / 8);
    let (mut px, mut py) = (bg_x % 8, bg_y % 8);

    // Entrada de mapa (16 bits) en el screen base block del fondo.
    let screen_base = ((bgcnt >> 8) & 0x1F) as usize * SCREEN_BLOCK_BYTES;
    let entry_off = screen_base + map_entry_index(tile_x, tile_y, width_tiles) * 2;
    let entry = u16::from_le_bytes([read_at(vram, entry_off), read_at(vram, entry_off + 1)]);

    let tile_num = (entry & MAP_TILE_MASK) as usize;
    if entry & MAP_HFLIP != 0 {
        px = 7 - px;
    }
    if entry & MAP_VFLIP != 0 {
        py = 7 - py;
    }

    let char_base = ((bgcnt >> 2) & 0b11) as usize * CHAR_BLOCK_BYTES;
    let index = if bgcnt & BGCNT_8BPP != 0 {
        // 8 bpp: un byte por píxel, paleta única de 256 colores.
        read_at(vram, char_base + tile_num * TILE_BYTES_8BPP + py * 8 + px) as usize
    } else {
        // 4 bpp: medio byte por píxel; el nibble bajo es el píxel par.
        let byte = read_at(vram, char_base + tile_num * TILE_BYTES_4BPP + py * 4 + px / 2);
        let nibble = if px & 1 == 0 { byte & 0x0F } else { byte >> 4 };
        nibble as usize
    };
    if index == 0 {
        return None; // índice 0 = transparente
    }
    // En 4 bpp el número de paleta (bits 12-15 de la entrada) selecciona un banco de
    // 16 colores; en 8 bpp se usa la paleta de fondo completa.
    let pal_index = if bgcnt & BGCNT_8BPP != 0 {
        index
    } else {
        ((entry >> 12) & 0xF) as usize * 16 + index
    };
    let off = pal_index * 2;
    Some(u16::from_le_bytes([read_at(pram, off), read_at(pram, off + 1)]))
}

/// Índice (en entradas de mapa de 16 bits) de la celda de tile `(tile_x, tile_y)`
/// dentro del *screen base block* de un fondo de texto, modelando el reparto en
/// *screenblocks* de 32×32 tiles según el ancho del fondo.
///
/// Los fondos de más de 256 px se componen de varios bloques de 32×32 entradas
/// (1024 cada uno): a la derecha y, en 512×512, también abajo. Esta función ubica la
/// celda en el bloque correcto (GBATEK, "Text BG Screen").
fn map_entry_index(tile_x: usize, tile_y: usize, width_tiles: usize) -> usize {
    let blocks_x = width_tiles / 32; // 1 o 2 bloques de ancho
    let block = (tile_x / 32) + (tile_y / 32) * blocks_x;
    block * 1024 + (tile_y % 32) * 32 + (tile_x % 32)
}

/// Tamaño de un fondo de texto en **tiles** (ancho, alto), de los bits 14-15 de
/// `BGxCNT`: 0 → 32×32 (256×256 px), 1 → 64×32, 2 → 32×64, 3 → 64×64.
fn bg_size_tiles(bgcnt: u16) -> (usize, usize) {
    match (bgcnt >> 14) & 0b11 {
        0 => (32, 32),
        1 => (64, 32),
        2 => (32, 64),
        _ => (64, 64),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{PRAM_SIZE, VRAM_SIZE};

    /// Comprueba que **todas** las líneas visibles del framebuffer son el color RGBA
    /// dado (helper para los tests de relleno).
    fn all_rows_are(ppu: &Ppu, rgba: [u8; 4]) -> bool {
        ppu.framebuffer()
            .chunks_exact(BYTES_PER_PIXEL)
            .all(|px| px == rgba)
    }

    /// El color RGBA del píxel `(x, y)` del framebuffer.
    fn pixel(ppu: &Ppu, x: usize, y: usize) -> [u8; 4] {
        let i = (y * SCREEN_WIDTH + x) * BYTES_PER_PIXEL;
        ppu.framebuffer()[i..i + 4].try_into().unwrap()
    }

    #[test]
    fn handles_reconoce_dispcnt_dispstat_y_vcount() {
        assert!(Ppu::handles(0x000)); // DISPCNT
        assert!(Ppu::handles(0x001));
        assert!(!Ppu::handles(0x002)); // green swap: no es nuestro
        assert!(Ppu::handles(0x004)); // DISPSTAT
        assert!(Ppu::handles(0x005));
        assert!(Ppu::handles(0x006)); // VCOUNT
        assert!(Ppu::handles(0x007));
        assert!(Ppu::handles(0x008)); // BG0CNT (2.4c)
        assert!(Ppu::handles(0x00F)); // BG3CNT (byte alto)
        assert!(Ppu::handles(0x010)); // BG0HOFS
        assert!(Ppu::handles(0x01F)); // BG3VOFS (byte alto)
        assert!(!Ppu::handles(0x020)); // ya fuera (BG2 afín BG2PA, llega en 2.4f)
    }

    #[test]
    fn dispcnt_almacena_y_devuelve_lo_escrito() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x07);
        ppu.write_u8(0x001, 0x12);
        assert_eq!(ppu.read_u8(0x000), 0x07);
        assert_eq!(ppu.read_u8(0x001), 0x12);
        assert_eq!(ppu.mode(), 7, "los bits 0-2 (0b111) son el modo de vídeo");
    }

    #[test]
    fn dispstat_solo_deja_escribir_los_enables_y_lyc() {
        let mut ppu = Ppu::new();
        // Intentar escribir los flags (bits 0-2) y los enables (3-5) y bits 6-7.
        ppu.write_u8(0x004, 0xFF);
        // Solo deben quedar los enables (bits 3-5 = 0x38); los flags 0-2 y los 6-7 a 0.
        assert_eq!(ppu.read_u8(0x004), 0x38, "solo bits 3-5 escribibles");
        // El byte alto es LYC, escribible entero.
        ppu.write_u8(0x005, 0x9C); // LYC = 156
        assert_eq!(ppu.read_u8(0x005), 0x9C);
    }

    #[test]
    fn vcount_es_de_solo_lectura() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x006, 0x55); // se ignora
        ppu.write_u8(0x007, 0x55);
        assert_eq!(ppu.read_u8(0x006), 0);
        assert_eq!(ppu.read_u8(0x007), 0);
    }

    #[test]
    fn enter_hblank_pone_el_flag_y_solicita_irq_si_esta_habilitada() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        irq.write_u8(0x200, 0xFF); // IE: habilita las 8 primeras fuentes (incl. H-Blank)

        // Sin enable de H-Blank IRQ: pone el flag pero no pide IRQ.
        ppu.enter_hblank(&mut irq);
        assert_ne!(ppu.read_u8(0x004) & 0x02, 0, "flag de H-Blank puesto");
        assert!(!irq.raised(), "sin enable de IRQ, no solicita");

        // Con el enable (bit 4) puesto, sí.
        ppu.write_u8(0x004, DISPSTAT_HBLANK_IRQ as u8);
        ppu.enter_hblank(&mut irq);
        assert!(irq.raised(), "con enable, solicita la IRQ de H-Blank");
    }

    #[test]
    fn enter_next_line_avanza_vcount_y_vuelve_a_cero_tras_la_ultima() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        for esperado in 1..TOTAL_SCANLINES {
            ppu.enter_next_line(&mut irq);
            assert_eq!(ppu.vcount(), esperado);
        }
        // Desde la 227, la siguiente vuelve a 0.
        ppu.enter_next_line(&mut irq);
        assert_eq!(ppu.vcount(), 0, "tras la línea 227 se reinicia el frame");
    }

    #[test]
    fn el_flag_de_vblank_se_activa_en_la_160_y_se_apaga_en_la_227() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        // Avanzar hasta la línea 159: aún visible, sin V-Blank.
        for _ in 0..159 {
            ppu.enter_next_line(&mut irq);
        }
        assert_eq!(ppu.vcount(), 159);
        assert_eq!(ppu.read_u8(0x004) & 0x01, 0, "la 159 no es V-Blank");
        // Línea 160: entra V-Blank.
        ppu.enter_next_line(&mut irq);
        assert_eq!(ppu.vcount(), 160);
        assert_ne!(ppu.read_u8(0x004) & 0x01, 0, "la 160 es V-Blank");
        // Avanzar hasta la 227: el flag se apaga (ya "sale" del V-Blank).
        for _ in 160..227 {
            ppu.enter_next_line(&mut irq);
        }
        assert_eq!(ppu.vcount(), 227);
        assert_eq!(ppu.read_u8(0x004) & 0x01, 0, "la 227 ya no marca V-Blank");
    }

    #[test]
    fn la_irq_de_vblank_se_solicita_al_entrar_en_la_linea_160() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        irq.write_u8(0x200, Interrupt::VBlank.bit() as u8); // IE = V-Blank
        ppu.write_u8(0x004, DISPSTAT_VBLANK_IRQ as u8); // enable de V-Blank IRQ
        for _ in 0..159 {
            ppu.enter_next_line(&mut irq);
        }
        assert!(!irq.raised(), "aún no se ha entrado en V-Blank");
        let entered = ppu.enter_next_line(&mut irq); // línea 160
        assert!(entered, "enter_next_line señala la entrada en V-Blank");
        assert!(irq.raised(), "la IRQ de V-Blank se solicitó en la 160");
    }

    #[test]
    fn la_irq_de_vcounter_se_solicita_en_la_linea_lyc() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        irq.write_u8(0x200, Interrupt::VCounter.bit() as u8); // IE = V-Counter
        ppu.write_u8(0x005, 5); // LYC = 5
        ppu.write_u8(0x004, DISPSTAT_VCOUNT_IRQ as u8); // enable de V-Counter IRQ
        for _ in 0..4 {
            ppu.enter_next_line(&mut irq);
        }
        assert!(!irq.raised(), "todavía no es la línea 5");
        ppu.enter_next_line(&mut irq); // línea 5
        assert_eq!(ppu.vcount(), 5);
        assert_ne!(ppu.read_u8(0x004) & 0x04, 0, "flag de coincidencia puesto");
        assert!(irq.raised(), "la IRQ de V-Counter se solicitó en la línea LYC");
    }

    #[test]
    fn conversion_de_color_bgr555_casos_clave() {
        assert_eq!(bgr555_to_rgba(0x0000), [0x00, 0x00, 0x00, 0xFF]); // negro
        assert_eq!(bgr555_to_rgba(0x7FFF), [0xFF, 0xFF, 0xFF, 0xFF]); // blanco
        assert_eq!(bgr555_to_rgba(0x001F), [0xFF, 0x00, 0x00, 0xFF]); // rojo
        assert_eq!(bgr555_to_rgba(0x03E0), [0x00, 0xFF, 0x00, 0xFF]); // verde
        assert_eq!(bgr555_to_rgba(0x7C00), [0x00, 0x00, 0xFF, 0xFF]); // azul
        assert_eq!(bgr555_to_rgba(0x8000), [0x00, 0x00, 0x00, 0xFF]); // bit 15 ignorado
    }

    #[test]
    fn render_scanline_modo3_pinta_solo_su_fila() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x03); // modo 3
        let mut vram = vec![0u8; VRAM_SIZE];
        let pram = vec![0u8; PRAM_SIZE];
        // Fila 1: primer píxel rojo (0x001F).
        let off = SCREEN_WIDTH * 2;
        vram[off..off + 2].copy_from_slice(&0x001Fu16.to_le_bytes());

        ppu.render_scanline(1, &vram, &pram);

        assert_eq!(pixel(&ppu, 0, 1), [0xFF, 0x00, 0x00, 0xFF], "fila 1 renderizada");
        // La fila 0 no se tocó: sigue como el framebuffer recién creado (ceros).
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0x00, 0x00, 0x00], "fila 0 intacta (sin renderizar)");
    }

    #[test]
    fn render_frame_completo_vuelca_la_vram_en_modo_3() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x03); // modo 3
        let mut vram = vec![0u8; VRAM_SIZE];
        let pram = vec![0u8; PRAM_SIZE];
        vram[0..2].copy_from_slice(&0x001Fu16.to_le_bytes()); // (0,0) rojo
        let last = (SCREEN_WIDTH * SCREEN_HEIGHT - 1) * 2;
        vram[last..last + 2].copy_from_slice(&0x7C00u16.to_le_bytes()); // último azul

        ppu.render_frame(&vram, &pram);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF]);
        assert_eq!(pixel(&ppu, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1), [0x00, 0x00, 0xFF, 0xFF]);
    }

    #[test]
    fn forced_blank_pinta_la_fila_blanca() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x03 | FORCED_BLANK as u8); // modo 3 + forced blank
        let mut vram = vec![0u8; VRAM_SIZE];
        vram[0..2].copy_from_slice(&0x001Fu16.to_le_bytes()); // rojo (debe ignorarse)
        let pram = vec![0u8; PRAM_SIZE];
        ppu.render_scanline(0, &vram, &pram);
        assert_eq!(pixel(&ppu, 0, 0), WHITE, "el forced blank ignora la VRAM");
    }

    #[test]
    fn un_modo_no_implementado_pinta_el_backdrop() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x02); // modo 2 (solo fondos afines, aún sin implementar)
        let vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        pram[0..2].copy_from_slice(&0x03E0u16.to_le_bytes()); // backdrop verde
        ppu.render_frame(&vram, &pram);
        assert!(all_rows_are(&ppu, [0x00, 0xFF, 0x00, 0xFF]), "todo el backdrop verde");
    }

    #[test]
    fn clear_framebuffer_rellena_todo() {
        let mut ppu = Ppu::new();
        ppu.clear_framebuffer([10, 20, 30, 0xFF]);
        assert!(all_rows_are(&ppu, [10, 20, 30, 0xFF]));
    }

    // ---- Fondos de tiles (Mini-Hito 2.4c) -----------------------------------

    #[test]
    fn bgcnt_almacena_los_cuatro_y_es_legible() {
        let mut ppu = Ppu::new();
        for bg in 0..4u32 {
            let off = 0x008 + bg * 2;
            ppu.write_u8(off, 0xCD);
            ppu.write_u8(off + 1, 0xAB);
            assert_eq!(ppu.read_u8(off), 0xCD, "BG{bg}CNT byte bajo");
            assert_eq!(ppu.read_u8(off + 1), 0xAB, "BG{bg}CNT byte alto");
        }
    }

    #[test]
    fn los_scroll_son_de_solo_escritura() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x010, 0x34); // BG0HOFS
        ppu.write_u8(0x011, 0x01);
        ppu.write_u8(0x012, 0x78); // BG0VOFS
        // Se almacenan internamente pero no son legibles (write-only → 0).
        assert_eq!(ppu.read_u8(0x010), 0);
        assert_eq!(ppu.read_u8(0x012), 0);
    }

    /// Escribe en `vram` la fila 0 de un tile 4 bpp con todos sus 8 píxeles al índice
    /// de paleta `pal_index` (1–15). `tile_num` y `char_base` (en bytes) ubican el tile.
    fn poner_fila0_tile_4bpp(vram: &mut [u8], char_base: usize, tile_num: usize, pal_index: u8) {
        let base = char_base + tile_num * 32; // 32 bytes por tile en 4 bpp
        let nibble_par = pal_index; // píxel par = nibble bajo
        let byte = nibble_par | (pal_index << 4); // dos píxeles por byte
        for b in 0..4 {
            vram[base + b] = byte;
        }
    }

    /// Programa un color BGR555 en la entrada `i` de la paleta de fondo (`PRAM`).
    fn poner_color_pram(pram: &mut [u8], i: usize, color: u16) {
        pram[i * 2..i * 2 + 2].copy_from_slice(&color.to_le_bytes());
    }

    #[test]
    fn modo0_dibuja_un_tile_y_respeta_la_transparencia() {
        let mut ppu = Ppu::new();
        // Modo 0, BG0 habilitado (bit 8).
        ppu.write_u8(0x000, 0x00);
        ppu.write_u8(0x001, 0x01);
        // BG0CNT: char base 0, screen base block 8 (0x4000), prioridad 0, 4 bpp, size 0.
        ppu.write_u8(0x008, 0x00);
        ppu.write_u8(0x009, 0x08);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // Tile 1: fila 0 toda al índice 1. Mapa: celda (0,0) → tile 1; celda (1,0) → 0.
        poner_fila0_tile_4bpp(&mut vram, 0, 1, 1);
        vram[0x4000..0x4002].copy_from_slice(&0x0001u16.to_le_bytes()); // celda (0,0) = tile 1
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_pram(&mut pram, 1, 0x001F); // paleta 1 = rojo

        ppu.render_scanline(0, &vram, &pram);

        // Los 8 píxeles del tile 1 son rojos.
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "píxel del tile 1");
        assert_eq!(pixel(&ppu, 7, 0), [0xFF, 0x00, 0x00, 0xFF]);
        // La celda (1,0) apunta al tile 0 (transparente) → se ve el backdrop.
        assert_eq!(pixel(&ppu, 8, 0), [0x00, 0x00, 0xFF, 0xFF], "tile 0 transparente → backdrop");
    }

    #[test]
    fn el_scroll_horizontal_desplaza_el_fondo() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x00);
        ppu.write_u8(0x001, 0x01); // modo 0, BG0
        ppu.write_u8(0x009, 0x08); // BG0CNT: screen base block 8
        ppu.write_u8(0x010, 8); // BG0HOFS = 8: la celda (1,0) pasa a empezar en x=0

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        poner_fila0_tile_4bpp(&mut vram, 0, 1, 1);
        // Tile 1 en la celda (1,0); la (0,0) queda vacía.
        vram[0x4002..0x4004].copy_from_slice(&0x0001u16.to_le_bytes());
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop
        poner_color_pram(&mut pram, 1, 0x001F); // rojo

        ppu.render_scanline(0, &vram, &pram);

        // Con hofs=8, el tile de la celda (1,0) se ve ya en x=0.
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "scroll trae el tile (1,0) a x=0");
    }

    #[test]
    fn la_prioridad_decide_que_fondo_se_ve() {
        let mut ppu = Ppu::new();
        // Modo 0, BG0 y BG1 habilitados (bits 8 y 9).
        ppu.write_u8(0x000, 0x00);
        ppu.write_u8(0x001, 0x03);
        // BG0CNT: prioridad 1 (bits 0-1 = 1), screen base block 8.
        ppu.write_u8(0x008, 0x01);
        ppu.write_u8(0x009, 0x08);
        // BG1CNT: prioridad 0 (más al frente), screen base block 9 (0x4800).
        ppu.write_u8(0x00A, 0x00);
        ppu.write_u8(0x00B, 0x09);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // Ambos fondos usan tiles distintos con colores distintos en la celda (0,0).
        poner_fila0_tile_4bpp(&mut vram, 0, 1, 1); // tile 1 → índice 1 (rojo)
        poner_fila0_tile_4bpp(&mut vram, 0, 2, 2); // tile 2 → índice 2 (verde)
        vram[0x4000..0x4002].copy_from_slice(&0x0001u16.to_le_bytes()); // BG0 celda (0,0) = tile 1
        vram[0x4800..0x4802].copy_from_slice(&0x0002u16.to_le_bytes()); // BG1 celda (0,0) = tile 2
        poner_color_pram(&mut pram, 0, 0x0000);
        poner_color_pram(&mut pram, 1, 0x001F); // rojo (BG0)
        poner_color_pram(&mut pram, 2, 0x03E0); // verde (BG1)

        ppu.render_scanline(0, &vram, &pram);

        // BG1 tiene prioridad 0 (delante de BG0, prioridad 1) → se ve verde.
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0xFF, 0x00, 0xFF], "gana la menor prioridad (BG1)");
    }

    #[test]
    fn el_volteo_horizontal_de_la_entrada_de_mapa_se_aplica() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x00);
        ppu.write_u8(0x001, 0x01); // modo 0, BG0
        ppu.write_u8(0x009, 0x08); // screen base block 8

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // Tile 1, fila 0: solo el píxel 0 (nibble bajo del primer byte) al índice 1.
        vram[32] = 0x01; // px0 = índice 1, px1 = 0
        // Celda (0,0) = tile 1 con volteo horizontal (bit 10).
        vram[0x4000..0x4002].copy_from_slice(&(0x0001u16 | MAP_HFLIP).to_le_bytes());
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_pram(&mut pram, 1, 0x001F); // rojo

        ppu.render_scanline(0, &vram, &pram);

        // Sin volteo el píxel rojo estaría en x=0; con volteo horizontal pasa a x=7.
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0x00, 0xFF, 0xFF], "x=0 ya no es el píxel del tile");
        assert_eq!(pixel(&ppu, 7, 0), [0xFF, 0x00, 0x00, 0xFF], "el píxel rojo se volteó a x=7");
    }

    #[test]
    fn modo0_en_8bpp_usa_la_paleta_de_256_colores() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x00);
        ppu.write_u8(0x001, 0x01); // modo 0, BG0
        // BG0CNT: 8 bpp (bit 7), screen base block 8.
        ppu.write_u8(0x008, BGCNT_8BPP as u8);
        ppu.write_u8(0x009, 0x08);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // 8 bpp: 64 bytes por tile, un byte por píxel. Tile 1, px0 = índice 5.
        vram[64] = 5;
        vram[0x4000..0x4002].copy_from_slice(&0x0001u16.to_le_bytes()); // celda (0,0) = tile 1
        poner_color_pram(&mut pram, 0, 0x0000);
        poner_color_pram(&mut pram, 5, 0x001F); // índice 5 = rojo

        ppu.render_scanline(0, &vram, &pram);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "8 bpp resuelve por índice directo");
    }

    #[test]
    fn map_entry_index_ubica_los_screenblocks() {
        // 256×256 (32×32): un solo bloque, índice lineal.
        assert_eq!(map_entry_index(0, 0, 32), 0);
        assert_eq!(map_entry_index(31, 31, 32), 31 * 32 + 31);
        // 512×256 (64×32): la columna 32 cae en el segundo screenblock (+1024).
        assert_eq!(map_entry_index(32, 0, 64), 1024);
        // 512×512 (64×64): fila 32 → bloque de abajo; con ancho 2 bloques, +2048.
        assert_eq!(map_entry_index(0, 32, 64), 2048);
    }

    #[test]
    fn can_wake_exige_enable_en_dispstat_y_en_ie() {
        let mut ppu = Ppu::new();
        let mut irq = InterruptControl::new();
        ppu.write_u8(0x004, DISPSTAT_VBLANK_IRQ as u8); // enable de V-Blank en DISPSTAT
        assert!(!ppu.can_wake(&irq), "sin IE no puede despertar");
        irq.write_u8(0x200, Interrupt::VBlank.bit() as u8); // IE = V-Blank
        assert!(ppu.can_wake(&irq), "con ambos enables, sí");
    }
}
