//! La **PPU** (Picture Processing Unit): el subsistema gráfico de la GBA.
//! Mini-Hito **2.4b — Renderizado por Scanlines**.
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
//! Este hito mantiene el único modo dibujable de 2.4a (el **modo 3** bitmap), ahora
//! por scanlines; el resto de bits de `DISPCNT` se siguen almacenando sin efecto.
//! Los modos de *tiles* (0/1/2) son el 2.4c, los sprites el 2.4d y los modos bitmap
//! 4/5 el 2.4e.

use crate::interrupt::{Interrupt, InterruptControl};
use crate::{BYTES_PER_PIXEL, FRAMEBUFFER_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH};

// ---- Registros y sus offsets (dentro de la región de I/O, base 0x0400_0000) ----

/// `DISPCNT` (control de pantalla, 16 bits): bytes `0x000`–`0x001`.
const DISPCNT_LO: u32 = 0x000;
/// `DISPSTAT` (estado/control del barrido, 16 bits): bytes `0x004`–`0x005`.
const DISPSTAT_LO: u32 = 0x004;
/// `VCOUNT` (línea actual, 16 bits, solo lectura): bytes `0x006`–`0x007`.
const VCOUNT_LO: u32 = 0x006;

/// Máscara del **modo de vídeo** en `DISPCNT` (bits 0-2).
const BG_MODE_MASK: u16 = 0b111;
/// Bit de ***forced blank*** en `DISPCNT` (bit 7): pantalla en blanco.
const FORCED_BLANK: u16 = 1 << 7;

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
            framebuffer: vec![0; FRAMEBUFFER_SIZE],
        }
    }

    /// `true` si el offset de I/O `io_off` cae en un registro que gestiona la PPU
    /// (`DISPCNT`, `DISPSTAT` o `VCOUNT`). Lo usa el bus para enrutar aquí el acceso.
    /// El hueco `0x002`–`0x003` (*green swap*, no implementado) queda fuera.
    pub fn handles(io_off: u32) -> bool {
        (DISPCNT_LO..DISPCNT_LO + 2).contains(&io_off)
            || (DISPSTAT_LO..VCOUNT_LO + 2).contains(&io_off)
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
    /// modo 3 → bitmap directo 16bpp de la VRAM; cualquier otro modo (aún sin
    /// implementar) → el *backdrop* (`PRAM[0]`).
    pub fn render_scanline(&mut self, y: u16, vram: &[u8], pram: &[u8]) {
        let y = y as usize;
        if y >= SCREEN_HEIGHT {
            return;
        }
        let start = y * SCREEN_WIDTH * BYTES_PER_PIXEL;
        let row = &mut self.framebuffer[start..start + SCREEN_WIDTH * BYTES_PER_PIXEL];

        if self.dispcnt & FORCED_BLANK != 0 {
            fill_row(row, WHITE);
            return;
        }
        match (self.dispcnt & BG_MODE_MASK) as u8 {
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
        assert!(!Ppu::handles(0x008)); // ya fuera (BG0CNT, llega en 2.4c)
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
        ppu.write_u8(0x000, 0x00); // modo 0 (tiles, aún sin implementar)
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
