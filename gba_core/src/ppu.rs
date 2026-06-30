//! La **PPU** (Picture Processing Unit): el subsistema gráfico de la GBA.
//! Mini-Hitos **2.4a–2.4f** (bitmap modo 3 → barrido por scanlines → fondos de tiles
//! → sprites/OAM → modos bitmap 4 y 5 con doble buffer → fondos y sprites afines).
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
//! Tras 2.4f esta PPU dibuja los **fondos de texto** de los modos 0 y 1, los **fondos
//! afines** (BG2 del modo 1; BG2/BG3 del modo 2), los **modos bitmap** 3 (16bpp
//! directo), 4 (8bpp paletado con doble buffer) y 5 (16bpp 160×128 con doble buffer),
//! y la capa de **sprites** (OAM) **regulares y afines** (rotación/escalado), todo por
//! scanlines y compuesto por prioridad. Quedan los efectos (ventanas, blending,
//! mosaico) para 2.4g–2.4h. El resto de bits de `DISPCNT`/`BGxCNT` no usados aquí
//! (p. ej. *mosaic*) se almacenan sin efecto.

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
/// Primer byte **después** del bloque de registros de scroll de fondo (`0x020`),
/// donde empieza el bloque afín.
const BG_REGS_END: u32 = 0x020;
/// Primer byte del bloque de **parámetros afines** de BG2/BG3 (`0x020`, Mini-Hito
/// 2.4f): por fondo, `PA`/`PB`/`PC`/`PD` (4×16 bits) + el punto de referencia `X`/`Y`
/// (2×32 bits) = 16 bytes; BG2 en `0x020`–`0x02F`, BG3 en `0x030`–`0x03F`. Todos de
/// **solo escritura**.
const AFFINE_REGS_START: u32 = 0x020;
/// Bytes por fondo afín en el bloque de parámetros (PA..PD + X + Y = 16).
const AFFINE_REGS_PER_BG: u32 = 0x010;
/// Primer byte **después** del bloque afín (`0x040`, donde empezarán las ventanas 2.4g).
const AFFINE_REGS_END: u32 = AFFINE_REGS_START + 2 * AFFINE_REGS_PER_BG;

/// Máscara del **modo de vídeo** en `DISPCNT` (bits 0-2).
const BG_MODE_MASK: u16 = 0b111;
/// Bit de **selección de frame** en `DISPCNT` (bit 4): en los modos bitmap con doble
/// buffer (4 y 5), elige qué *frame* se muestra — 0 = frame 0 (VRAM `0x0600_0000`),
/// 1 = frame 1 (`0x0600_A000`). Los modos 0–3 lo ignoran.
const DISPCNT_FRAME_SELECT: u16 = 1 << 4;
/// Bit de ***forced blank*** en `DISPCNT` (bit 7): pantalla en blanco.
const FORCED_BLANK: u16 = 1 << 7;
/// Bit del *enable* del fondo 0 en `DISPCNT` (bit 8). Los fondos BG0–BG3 ocupan los
/// bits 8-11: el fondo `n` se muestra si su bit `8 + n` está a 1.
const DISPCNT_BG0_ENABLE: u16 = 1 << 8;
/// Bit de **mapeo 1D** de los tiles de sprites en `DISPCNT` (bit 6): 1 = los tiles de
/// un sprite van consecutivos en VRAM; 0 = mapeo 2D (rejilla de 32×32 tiles).
const DISPCNT_OBJ_1D: u16 = 1 << 6;
/// Bit de *enable* de los **sprites** (OBJ) en `DISPCNT` (bit 12): 1 = se dibujan.
const DISPCNT_OBJ_ENABLE: u16 = 1 << 12;

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
/// Bit de **envoltura del área de display** en `BGxCNT` (bit 13, solo fondos afines,
/// Mini-Hito 2.4f): 1 = el fondo se repite (envuelve) fuera de su área; 0 = lo de
/// fuera es transparente.
const BGCNT_AFFINE_WRAP: u16 = 1 << 13;
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

// ---- Modos bitmap con doble buffer (Mini-Hito 2.4e) -------------------------

/// Tamaño en bytes de **un** *frame* de los modos bitmap con doble buffer (4 y 5):
/// 0xA000 (40 KiB). El frame 1 empieza en la VRAM a este offset (`0x0600_A000`).
const FRAME_BYTES: usize = 0xA000;
/// Ancho en píxeles del bitmap **reducido** del modo 5 (el resto de la pantalla, hasta
/// los 240 px, queda fuera de la imagen).
const MODE5_WIDTH: usize = 160;
/// Alto en píxeles del bitmap **reducido** del modo 5 (las líneas 128–159 quedan
/// fuera de la imagen).
const MODE5_HEIGHT: usize = 128;

// ---- Sprites / OAM (Mini-Hito 2.4d) ----

/// Offset (en bytes, dentro de la VRAM) del **bloque de tiles de los sprites**: los
/// OBJ leen sus tiles a partir de `0x0601_0000`, es decir, 64 KiB dentro de la VRAM.
const OBJ_TILE_VRAM_BASE: usize = 0x1_0000;
/// Offset (en bytes, dentro de la PRAM) de la **paleta de sprites**: las 256 entradas
/// de OBJ viven en la mitad alta de la PRAM (`0x0500_0200`).
const OBJ_PALETTE_BYTE_BASE: usize = 0x200;
/// Número de entradas de la OAM (128 sprites, 8 bytes cada uno).
const OAM_ENTRIES: usize = 128;
/// Granularidad del **número de tile** de un sprite (siempre en unidades de 32 bytes,
/// el tamaño de un tile 4 bpp), aunque el sprite sea 8 bpp.
const OBJ_TILE_UNIT_BYTES: usize = TILE_BYTES_4BPP;

// Campos de los atributos de un sprite (OAM, 3 × 16 bits por entrada).
/// `attr0` bits 0-7: coordenada **Y** (8 bits, con envoltura a 256).
const OBJ_ATTR0_Y_MASK: u16 = 0xFF;
/// `attr0` bit 8: sprite **afín** (rotación/escalado). Sin él, bit 9 = *disable*.
const OBJ_ATTR0_AFFINE: u16 = 1 << 8;
/// `attr0` bit 9 (en sprites no afines): sprite **deshabilitado** (no se dibuja).
const OBJ_ATTR0_DISABLE: u16 = 1 << 9;
/// `attr0` bit 9 (en sprites **afines**): **doble tamaño** del recuadro de dibujo
/// (Mini-Hito 2.4f). El mismo bit es "disable" en los no afines; lo decide el bit 8.
const OBJ_ATTR0_DOUBLE: u16 = 1 << 9;
/// `attr0` bit 13: profundidad de color **8 bpp** (256 colores); si es 0, 4 bpp.
const OBJ_ATTR0_8BPP: u16 = 1 << 13;
/// `attr0` bits 14-15: **forma** del sprite (cuadrado / horizontal / vertical).
const OBJ_ATTR0_SHAPE_SHIFT: u16 = 14;
/// `attr1` bits 0-8: coordenada **X** (9 bits, con envoltura a 512).
const OBJ_ATTR1_X_MASK: u16 = 0x1FF;
/// `attr1` bits 9-13 (sprites **afines**): índice del **grupo de parámetros afines**
/// (0–31) en la OAM (Mini-Hito 2.4f). En los no afines, los bits 12-13 son los volteos.
const OBJ_ATTR1_AFFINE_IDX_SHIFT: u16 = 9;
/// Máscara (tras el desplazamiento) del índice de grupo afín: 5 bits.
const OBJ_ATTR1_AFFINE_IDX_MASK: u16 = 0x1F;
/// `attr1` bit 12: **volteo horizontal** (sprites no afines).
const OBJ_ATTR1_HFLIP: u16 = 1 << 12;
/// `attr1` bit 13: **volteo vertical** (sprites no afines).
const OBJ_ATTR1_VFLIP: u16 = 1 << 13;
/// `attr1` bits 14-15: **tamaño** (junto con la forma de `attr0`, ver [`obj_size`]).
const OBJ_ATTR1_SIZE_SHIFT: u16 = 14;
/// `attr2` bits 0-9: **número de tile** base del sprite.
const OBJ_ATTR2_TILE_MASK: u16 = 0x3FF;
/// `attr2` bits 10-11: **prioridad** del sprite frente a los fondos (0 = más al frente).
const OBJ_ATTR2_PRIORITY_SHIFT: u16 = 10;
/// `attr2` bits 12-15: **banco de paleta** (4 bpp, 16 colores por banco).
const OBJ_ATTR2_PALBANK_SHIFT: u16 = 12;

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
    /// Parámetros de la **matriz afín** de BG2 y BG3 (`PA`/`PB`/`PC`/`PD`,
    /// `0x0400_0020`+, Mini-Hito 2.4f): punto fijo 8.8 con signo. Índice 0 = BG2,
    /// 1 = BG3. De **solo escritura**.
    bgpa: [i16; 2],
    bgpb: [i16; 2],
    bgpc: [i16; 2],
    bgpd: [i16; 2],
    /// Punto de **referencia** afín de BG2/BG3 (`BGxX`/`BGxY`, `0x0400_0028`+): 28 bits
    /// con signo en punto fijo 20.8. Guardado crudo (32 bits); se extiende de signo
    /// desde el bit 27 al usarlo. De **solo escritura**.
    bgx: [i32; 2],
    bgy: [i32; 2],
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
            bgpa: [0; 2],
            bgpb: [0; 2],
            bgpc: [0; 2],
            bgpd: [0; 2],
            bgx: [0; 2],
            bgy: [0; 2],
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
            || (AFFINE_REGS_START..AFFINE_REGS_END).contains(&io_off)
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
            // Bloque afín de BG2/BG3 (0x020–0x03F, solo escritura): por fondo, PA..PD
            // (4×16 bits) y X/Y (2×32 bits). Ver AFFINE_REGS_START.
            n if (AFFINE_REGS_START..AFFINE_REGS_END).contains(&n) => {
                let i = ((n - AFFINE_REGS_START) / AFFINE_REGS_PER_BG) as usize;
                let local = (n - AFFINE_REGS_START) % AFFINE_REGS_PER_BG;
                match local {
                    // PA/PB/PC/PD: cada uno 2 bytes, en este orden.
                    0..=7 => {
                        let reg = match local / 2 {
                            0 => &mut self.bgpa[i],
                            1 => &mut self.bgpb[i],
                            2 => &mut self.bgpc[i],
                            _ => &mut self.bgpd[i],
                        };
                        write_byte_i16(reg, local & 1, value);
                    }
                    // X: 4 bytes (0x08–0x0B). Y: 4 bytes (0x0C–0x0F).
                    8..=11 => write_byte_i32(&mut self.bgx[i], local - 8, value),
                    _ => write_byte_i32(&mut self.bgy[i], local - 12, value),
                }
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
    /// modos de **tiles** 0/1/2 (texto y afines, 2.4c/2.4f) → composición de fondos por
    /// prioridad; modos **bitmap** 3 (16bpp directo), 4 (8bpp paletado con doble buffer)
    /// y 5 (16bpp 160×128 con doble buffer, 2.4e) → el bitmap como BG2; un modo inválido
    /// (6/7) → el *backdrop* (`PRAM[0]`).
    ///
    /// > Los fondos de texto (modo 0: BG0–3; modo 1: BG0–1) y los **afines** (modo 1:
    /// > BG2; modo 2: BG2/BG3) se componen por prioridad, y sobre ellos la **capa de
    /// > sprites** (OAM, regulares y afines, Mini-Hitos 2.4d/2.4f).
    pub fn render_scanline(&mut self, y: u16, vram: &[u8], pram: &[u8], oam: &[u8]) {
        let y = y as usize;
        if y >= SCREEN_HEIGHT {
            return;
        }
        // Copiamos el estado de registros que necesita el render a locales antes de
        // tomar prestado el framebuffer mutablemente (evita un conflicto de préstamos
        // con `self`): todos son `Copy`.
        let dispcnt = self.dispcnt;
        let bg = self.bg_registers();

        // Capa de **sprites** de esta línea (Mini-Hito 2.4d): un color + prioridad por
        // píxel, o transparente. Es independiente del modo de fondo y se compone luego
        // sobre los fondos según prioridad. Solo se calcula si los OBJ están activos.
        let mut objs = [ObjPixel::TRANSPARENT; SCREEN_WIDTH];
        if dispcnt & DISPCNT_OBJ_ENABLE != 0 {
            render_obj_line(&mut objs, y, dispcnt, oam, vram, pram);
        }

        let start = y * SCREEN_WIDTH * BYTES_PER_PIXEL;
        let row = &mut self.framebuffer[start..start + SCREEN_WIDTH * BYTES_PER_PIXEL];

        if dispcnt & FORCED_BLANK != 0 {
            fill_row(row, WHITE);
            return;
        }
        match (dispcnt & BG_MODE_MASK) as u8 {
            mode @ 0..=2 => render_tiled_row(row, y, mode, dispcnt, &bg, &objs, vram, pram),
            3 => render_bitmap3_row(row, y, &bg.cnt, &objs, vram, pram),
            4 => render_bitmap4_row(row, y, dispcnt, &bg.cnt, &objs, vram, pram),
            5 => render_bitmap5_row(row, y, dispcnt, &bg.cnt, &objs, vram, pram),
            _ => render_backdrop_row(row, &objs, pram),
        }
    }

    /// Toma una **instantánea** (`Copy`) de los registros de fondo para el render de
    /// una scanline, evitando tomar prestado `self` mientras se escribe el framebuffer.
    fn bg_registers(&self) -> BgRegisters {
        BgRegisters {
            cnt: self.bgcnt,
            hofs: self.bghofs,
            vofs: self.bgvofs,
            pa: self.bgpa,
            pb: self.bgpb,
            pc: self.bgpc,
            pd: self.bgpd,
            x: self.bgx,
            y: self.bgy,
        }
    }

    /// Renderiza el **frame completo** de una vez (las 160 líneas visibles), como en
    /// 2.4a. Es el render **bajo demanda** que conservan el frontend para un volcado
    /// inmediato y los tests; el render por scanlines del bucle usa
    /// [`Ppu::render_scanline`] línea a línea.
    pub fn render_frame(&mut self, vram: &[u8], pram: &[u8], oam: &[u8]) {
        for y in 0..SCREEN_HEIGHT as u16 {
            self.render_scanline(y, vram, pram, oam);
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

/// Como [`write_byte`] pero sobre un registro de 16 bits **con signo** (un parámetro
/// afín `PA`/`PB`/`PC`/`PD`): opera sobre los bits crudos y reinterpreta.
fn write_byte_i16(reg: &mut i16, byte_sel: u32, value: u8) {
    let mut bits = *reg as u16;
    write_byte(&mut bits, byte_sel, value);
    *reg = bits as i16;
}

/// Escribe el byte `byte_idx` (0 = el menos significativo) de un registro de 32 bits
/// con signo (el punto de referencia afín `BGxX`/`BGxY`), conservando el resto.
fn write_byte_i32(reg: &mut i32, byte_idx: u32, value: u8) {
    let shift = byte_idx * 8;
    let mut bits = *reg as u32;
    bits = (bits & !(0xFFu32 << shift)) | (u32::from(value) << shift);
    *reg = bits as i32;
}

/// Instantánea (`Copy`) de los registros de fondo para el render de una scanline.
/// Agrupa los de **texto** (`cnt`/`hofs`/`vofs`, BG0–3) y los **afines** de BG2/BG3
/// (`pa`–`pd` matriz + `x`/`y` punto de referencia, indexados 0 = BG2, 1 = BG3).
#[derive(Clone, Copy)]
struct BgRegisters {
    cnt: [u16; 4],
    hofs: [u16; 4],
    vofs: [u16; 4],
    pa: [i16; 2],
    pb: [i16; 2],
    pc: [i16; 2],
    pd: [i16; 2],
    x: [i32; 2],
    y: [i32; 2],
}

/// Tipo de un fondo según el modo de vídeo: de **texto** (regular, con scroll y
/// volteos, 2.4c) o **afín** (matriz de rotación/escalado, siempre 8 bpp, 2.4f).
#[derive(Clone, Copy)]
enum BgKind {
    Text,
    Affine,
}

/// Renderiza una scanline `y` en los **modos de tiles** 0, 1 y 2, componiendo los
/// fondos activos por orden de prioridad sobre el *backdrop*.
///
/// Cada modo dibuja un conjunto de fondos, de **texto** o **afines** (Mini-Hito 2.4f):
/// modo 0 → BG0–3 de texto; modo 1 → BG0/1 de texto + **BG2 afín**; modo 2 → **BG2/3
/// afines**. De ellos se toman los habilitados en `DISPCNT` y se ordenan por
/// (prioridad, índice de fondo): para cada píxel gana el primer fondo —el más al
/// frente— que aporte un color no transparente (índice de paleta ≠ 0); si ninguno lo
/// hace, queda el *backdrop*. Sobre el resultado se compone la capa de sprites.
#[allow(clippy::too_many_arguments)]
fn render_tiled_row(
    row: &mut [u8],
    y: usize,
    mode: u8,
    dispcnt: u16,
    bg: &BgRegisters,
    objs: &[ObjPixel; SCREEN_WIDTH],
    vram: &[u8],
    pram: &[u8],
) {
    // Fondos que dibuja cada modo y de qué tipo (GBATEK).
    let layers: &[(usize, BgKind)] = match mode {
        0 => &[
            (0, BgKind::Text),
            (1, BgKind::Text),
            (2, BgKind::Text),
            (3, BgKind::Text),
        ],
        1 => &[(0, BgKind::Text), (1, BgKind::Text), (2, BgKind::Affine)],
        2 => &[(2, BgKind::Affine), (3, BgKind::Affine)],
        _ => &[],
    };
    // De ellos, los habilitados en DISPCNT (bits 8-11), ordenados por prioridad.
    let mut order = [(0usize, BgKind::Text); 4];
    let mut count = 0;
    for &(idx, kind) in layers {
        if dispcnt & (DISPCNT_BG0_ENABLE << idx) != 0 {
            order[count] = (idx, kind);
            count += 1;
        }
    }
    let order = &mut order[..count];
    // Estable y por (prioridad, índice): a igual prioridad, el fondo de menor índice
    // queda delante (regla de la GBA).
    order.sort_by_key(|&(idx, _)| ((bg.cnt[idx] & BGCNT_PRIORITY_MASK) as u8, idx as u8));

    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        // Fondo ganador del píxel: el primero (más al frente) con color no
        // transparente, junto a su prioridad para compararla con el sprite.
        let mut winner = None;
        for &(idx, kind) in order.iter() {
            let sample = match kind {
                BgKind::Text => {
                    sample_text_bg(bg.cnt[idx], bg.hofs[idx], bg.vofs[idx], x, y, vram, pram)
                }
                BgKind::Affine => sample_affine_bg(bg, idx, x, y, vram, pram),
            };
            if let Some(color) = sample {
                winner = Some((color, (bg.cnt[idx] & BGCNT_PRIORITY_MASK) as u8));
                break;
            }
        }
        out.copy_from_slice(&compose_pixel(winner, objs[x], pram));
    }
}

/// Muestrea el color de un **fondo afín** (BG2 o BG3, `idx` 2/3; los parámetros viven
/// indexados 0/1) en el píxel de pantalla `(screen_x, screen_y)`, aplicando la matriz
/// de rotación/escalado y el punto de referencia. Devuelve el color **BGR555**, o
/// `None` si es transparente (índice 0) o si el punto cae fuera del área sin envoltura.
///
/// El hardware mapea la pantalla a la textura con `(tx, ty) = P·(x, y) + ref`, con
/// `PA..PD` en punto fijo 8.8 y el punto de referencia en 20.8; el `>> 8` recupera el
/// píxel de textura. Los fondos afines son **siempre 8 bpp** y su mapa tiene **1 byte
/// por celda** (número de tile, sin volteos ni banco de paleta). Nunca panica.
fn sample_affine_bg(
    bg: &BgRegisters,
    idx: usize,
    screen_x: usize,
    screen_y: usize,
    vram: &[u8],
    pram: &[u8],
) -> Option<u16> {
    let a = idx - 2; // BG2 → 0, BG3 → 1
    let bgcnt = bg.cnt[idx];
    let (pa, pb) = (bg.pa[a] as i32, bg.pb[a] as i32);
    let (pc, pd) = (bg.pc[a] as i32, bg.pd[a] as i32);
    let refx = sign_extend_28(bg.x[a]);
    let refy = sign_extend_28(bg.y[a]);

    // Transformación afín (8.8 fijo): (x,y) de pantalla → (tx,ty) de textura.
    let (x, y) = (screen_x as i32, screen_y as i32);
    let mut ix = (pa * x + pb * y + refx) >> 8;
    let mut iy = (pc * x + pd * y + refy) >> 8;

    let size_px = 128i32 << ((bgcnt >> 14) & 0b11); // 128/256/512/1024 px (cuadrado)
    if bgcnt & BGCNT_AFFINE_WRAP != 0 {
        // Envoltura: el fondo se repite (tamaño es potencia de dos).
        ix &= size_px - 1;
        iy &= size_px - 1;
    } else if !(0..size_px).contains(&ix) || !(0..size_px).contains(&iy) {
        return None; // fuera del área y sin envoltura → transparente
    }
    let (ix, iy) = (ix as usize, iy as usize);
    let tiles_per_row = (size_px / 8) as usize;

    // Mapa afín: 1 byte por celda = número de tile.
    let screen_base = ((bgcnt >> 8) & 0x1F) as usize * SCREEN_BLOCK_BYTES;
    let tile_num = read_at(vram, screen_base + (iy / 8) * tiles_per_row + ix / 8) as usize;
    // Tiles afines: siempre 8 bpp, paleta de fondo de 256 colores.
    let char_base = ((bgcnt >> 2) & 0b11) as usize * CHAR_BLOCK_BYTES;
    let index = read_at(vram, char_base + tile_num * TILE_BYTES_8BPP + (iy % 8) * 8 + ix % 8);
    if index == 0 {
        return None; // índice 0 = transparente
    }
    let off = index as usize * 2;
    Some(u16::from_le_bytes([read_at(pram, off), read_at(pram, off + 1)]))
}

/// Extiende de signo un valor afín de **28 bits** (el punto de referencia `BGxX/Y`,
/// guardado en 32 bits) tomando el bit 27 como bit de signo.
#[inline]
fn sign_extend_28(v: i32) -> i32 {
    (v << 4) >> 4
}

/// Compone una scanline del **modo 3** (bitmap directo 16 bpp) con la capa de
/// sprites. El bitmap actúa como un único fondo (BG2) opaco, con la prioridad de
/// `BG2CNT`; los sprites se imponen según su propia prioridad.
fn render_bitmap3_row(
    row: &mut [u8],
    y: usize,
    bgcnt: &[u16; 4],
    objs: &[ObjPixel; SCREEN_WIDTH],
    vram: &[u8],
    pram: &[u8],
) {
    let bg_priority = (bgcnt[2] & BGCNT_PRIORITY_MASK) as u8;
    let row_base = y * SCREEN_WIDTH * 2;
    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        let off = row_base + x * 2;
        let color = u16::from_le_bytes([read_at(vram, off), read_at(vram, off + 1)]);
        out.copy_from_slice(&compose_pixel(Some((color, bg_priority)), objs[x], pram));
    }
}

/// Compone una scanline del **modo 4** (bitmap paletado de 8 bpp con doble buffer,
/// Mini-Hito 2.4e). Cada píxel de la VRAM es **un byte** = índice en la paleta de
/// fondo (`PRAM`); el índice **0 es transparente** (deja ver sprites/*backdrop*),
/// igual que un tile. El bit de *frame select* de `DISPCNT` elige cuál de los dos
/// *frames* (offset 0 o `0xA000`) se muestra. El bitmap actúa como BG2, con la
/// prioridad de `BG2CNT`.
fn render_bitmap4_row(
    row: &mut [u8],
    y: usize,
    dispcnt: u16,
    bgcnt: &[u16; 4],
    objs: &[ObjPixel; SCREEN_WIDTH],
    vram: &[u8],
    pram: &[u8],
) {
    let bg_priority = (bgcnt[2] & BGCNT_PRIORITY_MASK) as u8;
    let frame_base = if dispcnt & DISPCNT_FRAME_SELECT != 0 { FRAME_BYTES } else { 0 };
    let row_base = frame_base + y * SCREEN_WIDTH;
    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        let index = read_at(vram, row_base + x) as usize;
        // Índice 0 = transparente: el píxel del fondo no aporta nada (queda el sprite
        // o el backdrop). Cualquier otro índice lee su color de la paleta de fondo.
        let bg = if index == 0 {
            None
        } else {
            let off = index * 2;
            let color = u16::from_le_bytes([read_at(pram, off), read_at(pram, off + 1)]);
            Some((color, bg_priority))
        };
        out.copy_from_slice(&compose_pixel(bg, objs[x], pram));
    }
}

/// Compone una scanline del **modo 5** (bitmap directo 16 bpp **reducido** a 160×128
/// con doble buffer, Mini-Hito 2.4e). Igual que el modo 3 pero con menos resolución:
/// los píxeles fuera del recuadro 160×128 quedan fuera de la imagen y muestran el
/// *backdrop*. El bit de *frame select* de `DISPCNT` elige el *frame* (offset 0 o
/// `0xA000`). El bitmap actúa como BG2, con la prioridad de `BG2CNT`.
fn render_bitmap5_row(
    row: &mut [u8],
    y: usize,
    dispcnt: u16,
    bgcnt: &[u16; 4],
    objs: &[ObjPixel; SCREEN_WIDTH],
    vram: &[u8],
    pram: &[u8],
) {
    let bg_priority = (bgcnt[2] & BGCNT_PRIORITY_MASK) as u8;
    let frame_base = if dispcnt & DISPCNT_FRAME_SELECT != 0 { FRAME_BYTES } else { 0 };
    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        // Fuera del recuadro 160×128 no hay imagen: solo backdrop (y sprites encima).
        let bg = if x < MODE5_WIDTH && y < MODE5_HEIGHT {
            let off = frame_base + (y * MODE5_WIDTH + x) * 2;
            let color = u16::from_le_bytes([read_at(vram, off), read_at(vram, off + 1)]);
            Some((color, bg_priority))
        } else {
            None
        };
        out.copy_from_slice(&compose_pixel(bg, objs[x], pram));
    }
}

/// Compone una scanline de un modo **sin fondo dibujado** (un modo de vídeo inválido,
/// 6 o 7): solo el *backdrop* y la capa de sprites encima.
fn render_backdrop_row(row: &mut [u8], objs: &[ObjPixel; SCREEN_WIDTH], pram: &[u8]) {
    for (x, out) in row.chunks_exact_mut(BYTES_PER_PIXEL).enumerate() {
        out.copy_from_slice(&compose_pixel(None, objs[x], pram));
    }
}

/// Combina el píxel del **fondo** ganador (`bg`: color BGR555 + prioridad, o `None` si
/// es transparente) con el de la **capa de sprites** (`obj`). Un sprite opaco se
/// impone si su prioridad es **≤** la del fondo (a igualdad, el sprite va delante), o
/// si el fondo es transparente. Si nada opaco hay, queda el *backdrop* (`PRAM[0]`).
fn compose_pixel(bg: Option<(u16, u8)>, obj: ObjPixel, pram: &[u8]) -> [u8; 4] {
    let obj_wins = obj.opaque
        && match bg {
            Some((_, bg_prio)) => obj.priority <= bg_prio,
            None => true,
        };
    if obj_wins {
        return bgr555_to_rgba(obj.color);
    }
    match bg {
        Some((color, _)) => bgr555_to_rgba(color),
        None => backdrop(pram),
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

// ---- Sprites / OAM (Mini-Hito 2.4d) -----------------------------------------

/// Un píxel ya resuelto de la **capa de sprites** de una scanline: su color BGR555,
/// la prioridad del sprite (para compararla con la del fondo) y si es opaco. El valor
/// [`ObjPixel::TRANSPARENT`] es el de "ningún sprite aquí".
#[derive(Clone, Copy)]
struct ObjPixel {
    color: u16,
    priority: u8,
    opaque: bool,
}

impl ObjPixel {
    /// Píxel sin sprite: transparente y con la peor prioridad (detrás de todo).
    const TRANSPARENT: ObjPixel = ObjPixel { color: 0, priority: 4, opaque: false };
}

/// Rellena la capa de sprites `objs` con los OBJ visibles en la scanline `y`,
/// leyendo la OAM (atributos), la VRAM (tiles) y la PRAM (paleta de sprites) que el
/// bus presta. Recorre los 128 sprites en orden de índice OAM; a igualdad de píxel,
/// el de **menor índice** manda (se escribe solo si el píxel sigue transparente).
///
/// > Dibuja sprites **regulares** y **afines** (rotación/escalado, `attr0` bit 8,
/// > Mini-Hito 2.4f). No se modelan aún los modos de OBJ semitransparente / ventana
/// > (`attr0` bits 10-11) ni el *mosaic*.
fn render_obj_line(
    objs: &mut [ObjPixel; SCREEN_WIDTH],
    y: usize,
    dispcnt: u16,
    oam: &[u8],
    vram: &[u8],
    pram: &[u8],
) {
    let one_d = dispcnt & DISPCNT_OBJ_1D != 0;

    for i in 0..OAM_ENTRIES {
        let base = i * 8;
        let attr0 = read_u16_at(oam, base);
        let affine = attr0 & OBJ_ATTR0_AFFINE != 0;
        // El bit 9 es "disable" en los no afines (y "doble tamaño" en los afines).
        if !affine && attr0 & OBJ_ATTR0_DISABLE != 0 {
            continue;
        }
        let attr1 = read_u16_at(oam, base + 2);
        let attr2 = read_u16_at(oam, base + 4);

        let shape = (attr0 >> OBJ_ATTR0_SHAPE_SHIFT) & 0b11;
        let size = (attr1 >> OBJ_ATTR1_SIZE_SHIFT) & 0b11;
        let (w, h) = obj_size(shape, size);

        let y_coord = attr0 & OBJ_ATTR0_Y_MASK;
        let x_coord = (attr1 & OBJ_ATTR1_X_MASK) as usize;
        let eight_bpp = attr0 & OBJ_ATTR0_8BPP != 0;
        let tile_base = (attr2 & OBJ_ATTR2_TILE_MASK) as usize;
        let priority = ((attr2 >> OBJ_ATTR2_PRIORITY_SHIFT) & 0b11) as u8;
        let pal_bank = ((attr2 >> OBJ_ATTR2_PALBANK_SHIFT) & 0xF) as usize;

        if affine {
            // Recuadro de dibujo: el doble del sprite si el bit 9 (doble tamaño) está.
            let (bw, bh) = if attr0 & OBJ_ATTR0_DOUBLE != 0 { (w * 2, h * 2) } else { (w, h) };
            let row_in = ((y as u16).wrapping_sub(y_coord) & 0xFF) as usize;
            if row_in >= bh {
                continue;
            }
            // Matriz afín del grupo de parámetros seleccionado (attr1 bits 9-13).
            let group = ((attr1 >> OBJ_ATTR1_AFFINE_IDX_SHIFT) & OBJ_ATTR1_AFFINE_IDX_MASK) as usize;
            let (pa, pb, pc, pd) = obj_affine_params(oam, group);
            // Centros del recuadro y de la textura del sprite.
            let (hbw, hbh) = (bw as i32 / 2, bh as i32 / 2);
            let (hw, hh) = (w as i32 / 2, h as i32 / 2);
            let dy = row_in as i32 - hbh;
            for col in 0..bw {
                let screen_x = (x_coord + col) & 0x1FF;
                if screen_x >= SCREEN_WIDTH || objs[screen_x].opaque {
                    continue;
                }
                // Transforma el punto del recuadro (relativo al centro) a coordenada de
                // textura del sprite con la matriz inversa que guarda la OAM.
                let dx = col as i32 - hbw;
                let sx = ((pa * dx + pb * dy) >> 8) + hw;
                let sy = ((pc * dx + pd * dy) >> 8) + hh;
                if sx < 0 || sx >= w as i32 || sy < 0 || sy >= h as i32 {
                    continue; // el punto cae fuera del sprite real
                }
                let index =
                    obj_tile_index(sx as usize, sy as usize, w, tile_base, eight_bpp, one_d, vram);
                put_obj_pixel(objs, screen_x, index, eight_bpp, pal_bank, priority, pram);
            }
        } else {
            // Fila dentro del sprite (con envoltura a 256). Si cae fuera, no toca esta
            // línea: un sprite cerca del borde inferior "envuelve" arriba.
            let row_in = ((y as u16).wrapping_sub(y_coord) & 0xFF) as usize;
            if row_in >= h {
                continue;
            }
            let hflip = attr1 & OBJ_ATTR1_HFLIP != 0;
            let vflip = attr1 & OBJ_ATTR1_VFLIP != 0;
            let sy = if vflip { h - 1 - row_in } else { row_in };
            for col in 0..w {
                // Envoltura horizontal a 512 y recorte a la pantalla.
                let screen_x = (x_coord + col) & 0x1FF;
                if screen_x >= SCREEN_WIDTH || objs[screen_x].opaque {
                    continue;
                }
                let sx = if hflip { w - 1 - col } else { col };
                let index = obj_tile_index(sx, sy, w, tile_base, eight_bpp, one_d, vram);
                put_obj_pixel(objs, screen_x, index, eight_bpp, pal_bank, priority, pram);
            }
        }
    }
}

/// Resuelve el color de un **índice de tile** de sprite (`index`) y lo deposita en la
/// capa OBJ en `screen_x`, **solo si** el píxel sigue libre (el sprite de menor índice
/// OAM manda) y el índice no es 0 (transparente). 4 bpp toma el color de un banco de
/// 16; 8 bpp, de la paleta de sprites completa (segunda mitad de la PRAM).
fn put_obj_pixel(
    objs: &mut [ObjPixel; SCREEN_WIDTH],
    screen_x: usize,
    index: u8,
    eight_bpp: bool,
    pal_bank: usize,
    priority: u8,
    pram: &[u8],
) {
    if index == 0 || screen_x >= SCREEN_WIDTH || objs[screen_x].opaque {
        return;
    }
    let pal_index = if eight_bpp { index as usize } else { pal_bank * 16 + index as usize };
    let off = OBJ_PALETTE_BYTE_BASE + pal_index * 2;
    let color = u16::from_le_bytes([read_at(pram, off), read_at(pram, off + 1)]);
    objs[screen_x] = ObjPixel { color, priority, opaque: true };
}

/// Lee los cuatro parámetros de la **matriz afín** (`PA`,`PB`,`PC`,`PD`, en punto fijo
/// 8.8 con signo) del grupo `group` (0–31) de la OAM. Cada grupo ocupa 32 bytes (4
/// entradas de sprite) y guarda un parámetro en el **4º** *halfword* de cada entrada:
/// PA en `+0x06`, PB en `+0x0E`, PC en `+0x16`, PD en `+0x1E`.
fn obj_affine_params(oam: &[u8], group: usize) -> (i32, i32, i32, i32) {
    let base = group * 32;
    let p = |off: usize| read_u16_at(oam, base + off) as i16 as i32;
    (p(0x06), p(0x0E), p(0x16), p(0x1E))
}

/// Índice de paleta del píxel `(sx, sy)` **dentro** de un sprite (con los volteos ya
/// aplicados por el llamador), localizando su tile en la VRAM de OBJ según el mapeo
/// 1D/2D. El número de tile va siempre en unidades de 32 bytes; un tile 8 bpp ocupa
/// dos. Nunca panica (acceso por `read_at`).
fn obj_tile_index(
    sx: usize,
    sy: usize,
    width: usize,
    tile_base: usize,
    eight_bpp: bool,
    one_d: bool,
    vram: &[u8],
) -> u8 {
    let (cell_x, cell_y) = (sx / 8, sy / 8);
    let (tx, ty) = (sx % 8, sy % 8);
    let units_per_tile = if eight_bpp { 2 } else { 1 };
    let tile_units = if one_d {
        // 1D: los tiles del sprite van consecutivos (ancho/8 por fila).
        tile_base + (cell_y * (width / 8) + cell_x) * units_per_tile
    } else {
        // 2D: rejilla fija de 32 unidades de tile por fila en la VRAM de OBJ.
        tile_base + cell_y * 32 + cell_x * units_per_tile
    };
    let tile_addr = OBJ_TILE_VRAM_BASE + tile_units * OBJ_TILE_UNIT_BYTES;
    if eight_bpp {
        read_at(vram, tile_addr + ty * 8 + tx)
    } else {
        let byte = read_at(vram, tile_addr + ty * 4 + tx / 2);
        if tx & 1 == 0 { byte & 0x0F } else { byte >> 4 }
    }
}

/// Dimensiones en píxeles (ancho, alto) de un sprite a partir de su **forma**
/// (`attr0` bits 14-15) y **tamaño** (`attr1` bits 14-15), según la tabla de GBATEK.
fn obj_size(shape: u16, size: u16) -> (usize, usize) {
    match (shape, size) {
        (0, 0) => (8, 8),
        (0, 1) => (16, 16),
        (0, 2) => (32, 32),
        (0, 3) => (64, 64),
        (1, 0) => (16, 8),
        (1, 1) => (32, 8),
        (1, 2) => (32, 16),
        (1, 3) => (64, 32),
        (2, 0) => (8, 16),
        (2, 1) => (8, 32),
        (2, 2) => (16, 32),
        (2, 3) => (32, 64),
        // Forma 3 (prohibida): sin tamaño definido; tratamos como el mínimo.
        _ => (8, 8),
    }
}

/// Lee un valor de 16 bits *little-endian* de un buffer (OAM) sin panicar.
#[inline]
fn read_u16_at(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([read_at(buf, off), read_at(buf, off + 1)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{OAM_SIZE, PRAM_SIZE, VRAM_SIZE};

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
        assert!(Ppu::handles(0x020)); // BG2PA (afín, 2.4f)
        assert!(Ppu::handles(0x03F)); // BG3Y (byte alto)
        assert!(!Ppu::handles(0x040)); // ya fuera (WIN0H, ventanas en 2.4g)
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

        ppu.render_scanline(1, &vram, &pram, &[]);

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

        ppu.render_frame(&vram, &pram, &[]);

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
        ppu.render_scanline(0, &vram, &pram, &[]);
        assert_eq!(pixel(&ppu, 0, 0), WHITE, "el forced blank ignora la VRAM");
    }

    #[test]
    fn un_modo_invalido_pinta_el_backdrop() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x06); // modo 6 (inválido): sin fondo, solo backdrop
        let vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        pram[0..2].copy_from_slice(&0x03E0u16.to_le_bytes()); // backdrop verde
        ppu.render_frame(&vram, &pram, &[]);
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

        ppu.render_scanline(0, &vram, &pram, &[]);

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

        ppu.render_scanline(0, &vram, &pram, &[]);

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

        ppu.render_scanline(0, &vram, &pram, &[]);

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

        ppu.render_scanline(0, &vram, &pram, &[]);

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

        ppu.render_scanline(0, &vram, &pram, &[]);

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

    // ---- Sprites / OAM (Mini-Hito 2.4d) -------------------------------------

    /// Una OAM con los **128 sprites deshabilitados** (bit 9 de `attr0`). Es el punto
    /// de partida de los tests: una OAM a ceros sería en realidad 128 sprites válidos
    /// de 8×8 apilados en el origen (los juegos reales también desactivan los que no
    /// usan o los sacan de pantalla).
    fn oam_vacia() -> Vec<u8> {
        let mut oam = vec![0u8; OAM_SIZE];
        for i in 0..OAM_ENTRIES {
            oam[i * 8..i * 8 + 2].copy_from_slice(&OBJ_ATTR0_DISABLE.to_le_bytes());
        }
        oam
    }

    /// Escribe los tres atributos de un sprite en la OAM (el 4º hueco de cada entrada
    /// es parámetro afín, que este hito no usa).
    fn poner_sprite(oam: &mut [u8], i: usize, attr0: u16, attr1: u16, attr2: u16) {
        let base = i * 8;
        oam[base..base + 2].copy_from_slice(&attr0.to_le_bytes());
        oam[base + 2..base + 4].copy_from_slice(&attr1.to_le_bytes());
        oam[base + 4..base + 6].copy_from_slice(&attr2.to_le_bytes());
    }

    /// Programa un color en la entrada `i` de la **paleta de sprites** (PRAM `0x200`+).
    fn poner_color_obj_pram(pram: &mut [u8], i: usize, color: u16) {
        let off = OBJ_PALETTE_BYTE_BASE + i * 2;
        pram[off..off + 2].copy_from_slice(&color.to_le_bytes());
    }

    /// Escribe los cuatro parámetros afines (`PA`,`PB`,`PC`,`PD`) del grupo `group` en
    /// la OAM, en los huecos del 4º *halfword* de sus 4 entradas (ver `obj_affine_params`).
    fn poner_obj_affine(oam: &mut [u8], group: usize, pa: u16, pb: u16, pc: u16, pd: u16) {
        let base = group * 32;
        oam[base + 0x06..base + 0x08].copy_from_slice(&pa.to_le_bytes());
        oam[base + 0x0E..base + 0x10].copy_from_slice(&pb.to_le_bytes());
        oam[base + 0x16..base + 0x18].copy_from_slice(&pc.to_le_bytes());
        oam[base + 0x1E..base + 0x20].copy_from_slice(&pd.to_le_bytes());
    }

    /// DISPCNT con un modo, y opcionalmente OBJ habilitado y mapeo 1D.
    fn dispcnt(ppu: &mut Ppu, mode: u8, obj_enable: bool, one_d: bool) {
        let mut v = mode as u16;
        if obj_enable {
            v |= DISPCNT_OBJ_ENABLE;
        }
        if one_d {
            v |= DISPCNT_OBJ_1D;
        }
        ppu.write_u8(0x000, v as u8);
        ppu.write_u8(0x001, (v >> 8) as u8);
    }

    #[test]
    fn un_sprite_se_dibuja_sobre_el_backdrop() {
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true); // modo 0, OBJ on, 1D

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();

        // Tile 0 de OBJ (en 0x10000), fila 0 toda al índice 1.
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_obj_pram(&mut pram, 1, 0x001F); // paleta OBJ índice 1 = rojo
        // Sprite 0: 8×8 (forma 0/tamaño 0), X=0 Y=0, tile 0, prioridad 0.
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000);

        ppu.render_scanline(0, &vram, &pram, &oam);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "sprite rojo en x=0");
        assert_eq!(pixel(&ppu, 7, 0), [0xFF, 0x00, 0x00, 0xFF], "el sprite cubre 8 px");
        assert_eq!(pixel(&ppu, 8, 0), [0x00, 0x00, 0xFF, 0xFF], "fuera del sprite → backdrop");
    }

    #[test]
    fn el_sprite_respeta_su_posicion_x_e_y() {
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_obj_pram(&mut pram, 1, 0x001F); // rojo
        // Sprite en X=100, Y=50.
        poner_sprite(&mut oam, 0, 50, 100, 0x0000);

        // La fila 0 del sprite cae en la scanline 50; el píxel 0 del sprite en x=100.
        ppu.render_scanline(50, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 100, 50), [0xFF, 0x00, 0x00, 0xFF], "esquina del sprite");
        assert_eq!(pixel(&ppu, 99, 50), [0x00, 0x00, 0x00, 0xFF], "justo a la izquierda, nada");
        // Otra scanline distinta no toca el sprite (solo su fila 0 tiene datos).
        ppu.render_scanline(49, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 100, 49), [0x00, 0x00, 0x00, 0xFF], "la línea de arriba, nada");
    }

    #[test]
    fn la_prioridad_decide_entre_sprite_y_fondo() {
        // BG0 y un sprite caen en el mismo píxel; gana el de menor prioridad.
        let prep = || {
            let mut ppu = Ppu::new();
            // Modo 0, BG0 (bit 8) + OBJ (bit 12) + 1D (bit 6).
            ppu.write_u8(0x000, DISPCNT_OBJ_1D as u8);
            ppu.write_u8(0x001, ((DISPCNT_BG0_ENABLE | DISPCNT_OBJ_ENABLE) >> 8) as u8);
            ppu.write_u8(0x009, 0x08); // BG0CNT: screen base block 8

            let mut vram = vec![0u8; VRAM_SIZE];
            let mut pram = vec![0u8; PRAM_SIZE];
            let mut oam = oam_vacia();
            // Fondo: tile 1 (índice 1 = verde) en la celda (0,0).
            poner_fila0_tile_4bpp(&mut vram, 0, 1, 1);
            vram[0x4000..0x4002].copy_from_slice(&0x0001u16.to_le_bytes());
            poner_color_pram(&mut pram, 1, 0x03E0); // BG: verde
            // Sprite: tile 0 (índice 1 = rojo) en (0,0).
            poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
            poner_color_obj_pram(&mut pram, 1, 0x001F); // OBJ: rojo
            poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000);
            (ppu, vram, pram, oam)
        };

        // Sprite prioridad 0, BG0 prioridad 1 → gana el sprite (rojo).
        let (mut ppu, vram, pram, mut oam) = prep();
        ppu.write_u8(0x008, 0x01); // BG0CNT prioridad 1
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000); // OBJ prioridad 0
        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "sprite (prio 0) delante del BG (prio 1)");

        // Sprite prioridad 2, BG0 prioridad 0 → gana el fondo (verde).
        let (mut ppu, vram, pram, mut oam) = prep();
        ppu.write_u8(0x008, 0x00); // BG0CNT prioridad 0
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 2 << OBJ_ATTR2_PRIORITY_SHIFT); // OBJ prioridad 2
        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0xFF, 0x00, 0xFF], "BG (prio 0) delante del sprite (prio 2)");
    }

    #[test]
    fn el_volteo_horizontal_de_un_sprite_se_aplica() {
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        // Tile 0, fila 0: solo el píxel 0 (nibble bajo del primer byte) al índice 1.
        vram[OBJ_TILE_VRAM_BASE] = 0x01;
        poner_color_obj_pram(&mut pram, 1, 0x001F); // rojo
        // Sprite 8×8 en (0,0) con volteo horizontal (attr1 bit 12).
        poner_sprite(&mut oam, 0, 0x0000, OBJ_ATTR1_HFLIP, 0x0000);

        ppu.render_scanline(0, &vram, &pram, &oam);

        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0x00, 0x00, 0xFF], "x=0 ya no es el píxel del sprite");
        assert_eq!(pixel(&ppu, 7, 0), [0xFF, 0x00, 0x00, 0xFF], "el píxel se volteó a x=7");
    }

    #[test]
    fn un_sprite_8bpp_usa_la_paleta_directa() {
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        // 8 bpp: un byte por píxel. Tile 0, px0 = índice 5.
        vram[OBJ_TILE_VRAM_BASE] = 5;
        poner_color_obj_pram(&mut pram, 5, 0x001F); // índice 5 = rojo
        // Sprite 8×8, 8 bpp (attr0 bit 13), en (0,0).
        poner_sprite(&mut oam, 0, OBJ_ATTR0_8BPP, 0x0000, 0x0000);

        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "8 bpp resuelve por índice directo");
    }

    #[test]
    fn un_sprite_deshabilitado_o_con_obj_off_no_se_dibuja() {
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_obj_pram(&mut pram, 1, 0x001F); // rojo

        // (a) OBJ deshabilitado en DISPCNT: no se compone la capa de sprites.
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, false, true);
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000);
        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0x00, 0xFF, 0xFF], "OBJ off → solo backdrop");

        // (b) OBJ on, pero el sprite marcado como deshabilitado (attr0 bit 9).
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);
        poner_sprite(&mut oam, 0, OBJ_ATTR0_DISABLE, 0x0000, 0x0000);
        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0x00, 0x00, 0xFF, 0xFF], "sprite disable → solo backdrop");
    }

    #[test]
    fn un_sprite_afin_con_matriz_identidad_se_dibuja_como_uno_regular() {
        // Con la matriz afín identidad (PA=PD=0x100, PB=PC=0), un sprite afín debe
        // verse igual que un sprite regular (Mini-Hito 2.4f).
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_obj_pram(&mut pram, 1, 0x001F); // rojo
        // Grupo afín 0: identidad (PA=0x0100, PD=0x0100).
        poner_obj_affine(&mut oam, 0, 0x0100, 0x0000, 0x0000, 0x0100);
        // Sprite afín 8×8 en (0,0), grupo 0.
        poner_sprite(&mut oam, 0, OBJ_ATTR0_AFFINE, 0x0000, 0x0000);

        ppu.render_scanline(0, &vram, &pram, &oam);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "afín identidad = sprite rojo");
        assert_eq!(pixel(&ppu, 7, 0), [0xFF, 0x00, 0x00, 0xFF], "cubre los 8 px");
        assert_eq!(pixel(&ppu, 8, 0), [0x00, 0x00, 0xFF, 0xFF], "fuera → backdrop");
    }

    #[test]
    fn obj_size_cubre_las_formas_y_tamanos() {
        assert_eq!(obj_size(0, 0), (8, 8)); // cuadrado mínimo
        assert_eq!(obj_size(0, 3), (64, 64)); // cuadrado máximo
        assert_eq!(obj_size(1, 0), (16, 8)); // horizontal
        assert_eq!(obj_size(1, 3), (64, 32));
        assert_eq!(obj_size(2, 0), (8, 16)); // vertical
        assert_eq!(obj_size(2, 3), (32, 64));
    }

    #[test]
    fn el_menor_indice_de_oam_queda_delante() {
        // Dos sprites en el mismo píxel con la misma prioridad: gana el de índice OAM
        // menor (sprite 0 sobre sprite 1).
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        // Tile 0 → índice 1 (rojo); tile 1 → índice 1 pero banco de paleta 1 (verde).
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 1, 1);
        poner_color_obj_pram(&mut pram, 1, 0x001F); // banco 0, índice 1 = rojo
        poner_color_obj_pram(&mut pram, 16 + 1, 0x03E0); // banco 1, índice 1 = verde
        // Sprite 1 (verde, banco 1) y sprite 0 (rojo, banco 0), ambos en (0,0).
        poner_sprite(&mut oam, 1, 0x0000, 0x0000, 0x0001 | (1 << OBJ_ATTR2_PALBANK_SHIFT));
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000);
        ppu.render_scanline(0, &vram, &pram, &oam);
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "el sprite 0 (rojo) tapa al 1");
    }

    // ---- Modos bitmap 4 y 5 (Mini-Hito 2.4e) --------------------------------

    #[test]
    fn modo4_resuelve_el_color_por_la_paleta_y_respeta_el_indice_0() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x04); // modo 4
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // (0,1) → índice 3; (1,1) → índice 0 (transparente).
        vram[SCREEN_WIDTH] = 3; // y=1, x=0
        vram[SCREEN_WIDTH + 1] = 0; // y=1, x=1
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_pram(&mut pram, 3, 0x001F); // índice 3 = rojo

        ppu.render_scanline(1, &vram, &pram, &[]);

        assert_eq!(pixel(&ppu, 0, 1), [0xFF, 0x00, 0x00, 0xFF], "índice 3 → rojo");
        assert_eq!(pixel(&ppu, 1, 1), [0x00, 0x00, 0xFF, 0xFF], "índice 0 → backdrop");
    }

    #[test]
    fn modo4_el_frame_select_elige_el_segundo_buffer() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x04 | DISPCNT_FRAME_SELECT as u8); // modo 4, frame 1
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // El frame 0 (offset 0) tiene índice 2; el frame 1 (offset 0xA000) índice 1.
        vram[0] = 2;
        vram[FRAME_BYTES] = 1;
        poner_color_pram(&mut pram, 1, 0x001F); // rojo
        poner_color_pram(&mut pram, 2, 0x03E0); // verde

        ppu.render_scanline(0, &vram, &pram, &[]);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "frame 1 → lee el índice del 0xA000");
    }

    #[test]
    fn modo5_pinta_16bpp_dentro_del_recuadro_y_backdrop_fuera() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x05); // modo 5
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        poner_color_pram(&mut pram, 0, 0x03E0); // backdrop verde
        // (0,0) dentro del recuadro 160×128 = rojo directo.
        vram[0..2].copy_from_slice(&0x001Fu16.to_le_bytes());

        ppu.render_scanline(0, &vram, &pram, &[]);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "dentro del recuadro");
        // x=160 está fuera del ancho reducido → backdrop.
        assert_eq!(pixel(&ppu, MODE5_WIDTH, 0), [0x00, 0xFF, 0x00, 0xFF], "fuera del ancho → backdrop");
        // La línea 128 está fuera del alto reducido → backdrop en toda la fila.
        ppu.render_scanline(MODE5_HEIGHT as u16, &vram, &pram, &[]);
        assert_eq!(pixel(&ppu, 0, MODE5_HEIGHT), [0x00, 0xFF, 0x00, 0xFF], "fuera del alto → backdrop");
    }

    #[test]
    fn modo5_el_frame_select_elige_el_segundo_buffer() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x05 | DISPCNT_FRAME_SELECT as u8); // modo 5, frame 1
        let mut vram = vec![0u8; VRAM_SIZE];
        let pram = vec![0u8; PRAM_SIZE];
        // Frame 0 azul, frame 1 rojo en (0,0); debe verse el del frame 1.
        vram[0..2].copy_from_slice(&0x7C00u16.to_le_bytes());
        vram[FRAME_BYTES..FRAME_BYTES + 2].copy_from_slice(&0x001Fu16.to_le_bytes());

        ppu.render_scanline(0, &vram, &pram, &[]);

        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "frame 1 → rojo del 0xA000");
    }

    #[test]
    fn modo4_un_sprite_se_compone_sobre_el_bitmap() {
        // El bitmap del modo 4 es BG2; un sprite con mejor prioridad se impone.
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 4, true, true); // modo 4, OBJ on, 1D
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        vram[8] = 2; // bitmap (8,0) = índice 2 (justo fuera del sprite 8×8)
        poner_color_pram(&mut pram, 2, 0x03E0); // bitmap verde
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_obj_pram(&mut pram, 1, 0x001F); // sprite rojo
        poner_sprite(&mut oam, 0, 0x0000, 0x0000, 0x0000); // sprite 8×8 prioridad 0

        ppu.render_scanline(0, &vram, &pram, &oam);

        // El sprite 8×8 cubre x=0..7; el bitmap se ve a partir de x=8.
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "sprite (prio 0) sobre el bitmap");
        assert_eq!(pixel(&ppu, 8, 0), [0x00, 0xFF, 0x00, 0xFF], "fuera del sprite → bitmap verde");
    }

    // ---- Fondos y sprites afines (Mini-Hito 2.4f) ---------------------------

    /// Escribe los parámetros afines de un fondo (`bg` = 2 o 3) por sus registros de
    /// solo escritura (`0x020`+): matriz `PA`/`PB`/`PC`/`PD` (8.8) y referencia `X`/`Y`.
    #[allow(clippy::too_many_arguments)]
    fn poner_bg_afin(ppu: &mut Ppu, bg: u32, pa: i16, pb: i16, pc: i16, pd: i16, x: i32, y: i32) {
        let base = AFFINE_REGS_START + (bg - 2) * AFFINE_REGS_PER_BG;
        let mut w16 = |off: u32, v: u16| {
            ppu.write_u8(off, v as u8);
            ppu.write_u8(off + 1, (v >> 8) as u8);
        };
        w16(base, pa as u16);
        w16(base + 2, pb as u16);
        w16(base + 4, pc as u16);
        w16(base + 6, pd as u16);
        for b in 0..4 {
            ppu.write_u8(base + 8 + b, (x >> (b * 8)) as u8);
            ppu.write_u8(base + 12 + b, (y >> (b * 8)) as u8);
        }
    }

    /// Un fondo afín 8 bpp con matriz **identidad** se dibuja como un fondo normal;
    /// fuera de su área (128×128, sin envoltura) se ve el *backdrop*.
    #[test]
    fn modo2_dibuja_un_fondo_afin_con_identidad() {
        let mut ppu = Ppu::new();
        // Modo 2, BG2 habilitado (bit 10 → byte alto 0x04).
        ppu.write_u8(0x000, 0x02);
        ppu.write_u8(0x001, 0x04);
        // BG2CNT (0x00C): screen base block 1 (0x800), char base 0, tamaño 0 (128×128).
        ppu.write_u8(0x00D, 0x01);

        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // Mapa afín: celda (0,0) → tile 1 (1 byte por celda, en el screen base block).
        vram[0x800] = 1;
        // Tile 1 en 8 bpp: 64 bytes/tile; píxel (0,0) = índice 5.
        vram[64] = 5;
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_pram(&mut pram, 5, 0x001F); // índice 5 = rojo
        // Identidad: PA=PD=1.0 (0x100), sin traslación.
        poner_bg_afin(&mut ppu, 2, 0x0100, 0, 0, 0x0100, 0, 0);

        ppu.render_scanline(0, &vram, &pram, &[]);
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "afín identidad → tile rojo");
        // x=200 cae fuera del área de 128 px y sin envoltura → backdrop.
        assert_eq!(pixel(&ppu, 200, 0), [0x00, 0x00, 0xFF, 0xFF], "fuera del área → backdrop");
    }

    /// Con el bit de envoltura de `BGxCNT` (13), el fondo afín se repite fuera de su área.
    #[test]
    fn el_fondo_afin_envuelve_con_el_bit_de_wrap() {
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        // Mapa: todas las celdas de la primera fila de tiles → tile 1; tile 1 todo a 5.
        for c in 0..16 {
            vram[0x800 + c] = 1;
        }
        for b in 0..64 {
            vram[64 + b] = 5;
        }
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_pram(&mut pram, 5, 0x001F); // rojo

        // Referencia X = -1 píxel (ix = -1): sin envoltura cae fuera; con ella, repite.
        let prep = |wrap: bool| {
            let mut ppu = Ppu::new();
            ppu.write_u8(0x000, 0x02);
            ppu.write_u8(0x001, 0x04); // BG2 enable
            let mut cnt = 0x01u16 << 8; // screen base block 1
            if wrap {
                cnt |= BGCNT_AFFINE_WRAP;
            }
            ppu.write_u8(0x00C, cnt as u8);
            ppu.write_u8(0x00D, (cnt >> 8) as u8);
            poner_bg_afin(&mut ppu, 2, 0x0100, 0, 0, 0x0100, -256, 0); // refx = -1.0
            ppu
        };

        let mut sin = prep(false);
        sin.render_scanline(0, &vram, &pram, &[]);
        assert_eq!(pixel(&sin, 0, 0), [0x00, 0x00, 0xFF, 0xFF], "sin wrap, ix=-1 → backdrop");

        let mut con = prep(true);
        con.render_scanline(0, &vram, &pram, &[]);
        assert_eq!(pixel(&con, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "con wrap, ix=-1 envuelve → tile");
    }

    /// Un sprite afín con doble tamaño y la matriz a 0.5 (PA=PD=0x80) se dibuja al
    /// **doble** de su tamaño (zoom 2×): un 8×8 ocupa 16 px de pantalla.
    #[test]
    fn un_sprite_afin_escala_al_doble() {
        let mut ppu = Ppu::new();
        dispcnt(&mut ppu, 0, true, true);
        let mut vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        let mut oam = oam_vacia();
        // Tile 0 fila 0 todo al índice 1 (rojo).
        poner_fila0_tile_4bpp(&mut vram, OBJ_TILE_VRAM_BASE, 0, 1);
        poner_color_pram(&mut pram, 0, 0x7C00); // backdrop azul
        poner_color_obj_pram(&mut pram, 1, 0x001F); // rojo
        // Grupo 0: PA=PD=0x80 (0.5 → cada píxel de pantalla avanza media textura → 2×).
        poner_obj_affine(&mut oam, 0, 0x0080, 0x0000, 0x0000, 0x0080);
        // Sprite afín 8×8 con doble tamaño (recuadro 16×16) en (0,0), grupo 0.
        poner_sprite(&mut oam, 0, OBJ_ATTR0_AFFINE | OBJ_ATTR0_DOUBLE, 0x0000, 0x0000);

        ppu.render_scanline(0, &vram, &pram, &oam);
        // La fila 0 del tile (roja) se estira por los 16 px del recuadro.
        assert_eq!(pixel(&ppu, 0, 0), [0xFF, 0x00, 0x00, 0xFF], "zoom 2×: x=0 rojo");
        assert_eq!(pixel(&ppu, 15, 0), [0xFF, 0x00, 0x00, 0xFF], "zoom 2×: x=15 aún dentro");
        assert_eq!(pixel(&ppu, 16, 0), [0x00, 0x00, 0xFF, 0xFF], "x=16 ya fuera → backdrop");
    }
}
