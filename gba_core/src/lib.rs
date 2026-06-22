//! # gba_core — el núcleo del emulador de Game Boy Advance
//!
//! Esta crate es una **librería pura**: no depende de ninguna librería gráfica,
//! de ventanas ni de entrada. Su única salida visual es un *framebuffer* crudo
//! en formato **RGBA** (240 × 160 × 4 bytes) que el frontend (por ejemplo
//! `gba_desktop`) se encarga de pintar en pantalla.
//!
//! Mantener esta separación desde el día 1 es lo que permitirá, más adelante,
//! sustituir el frontend de escritorio por uno de Android, iOS o WASM sin tocar
//! una sola línea del núcleo.
//!
//! ## Estado actual (Fase 2.1c-bis)
//!
//! Además de cargar y validar el cartucho (Fase 1), el núcleo tiene el
//! esqueleto del hardware: la CPU ARM7TDMI ([`Cpu`]) con sus registros y modos,
//! y el bus de memoria ([`Bus`]) con el mapa de memoria de la consola. La
//! consola [`Gba`] ya integra ambos: [`Gba::with_cartridge`] vuelca la ROM en
//! el bus y coloca el `PC` en el arranque, [`Gba::fetch`] realiza el **"Fetch"**
//! —leer la instrucción a la que apunta el `PC`— y el **"Decode"** identifica el
//! tipo de instrucción: [`Gba::decode_arm`] para el modo ARM (flujo de dos pasos
//! condición → opcode) y [`Gba::decode_thumb`] para el modo THUMB (16 bits, con
//! un decoder separado). Todavía no se ejecuta su lógica; eso llega en el
//! Mini-Hito 2.1d. La frontera con el frontend —entregar un buffer RGBA— no
//! cambiará.

pub mod arm;
pub mod bus;
pub mod cartridge;
pub mod cpu;
pub mod header;
pub mod thumb;

pub use arm::{ArmInstruction, Condition, Decoded};
pub use bus::Bus;
pub use cartridge::{Cartridge, CartridgeError, MAX_ROM_SIZE, MIN_ROM_SIZE};
pub use cpu::{Cpu, CpuMode, Cpsr};
pub use header::Header;
pub use thumb::ThumbInstruction;

/// Anchura de la pantalla de la GBA, en píxeles.
pub const SCREEN_WIDTH: usize = 240;

/// Altura de la pantalla de la GBA, en píxeles.
pub const SCREEN_HEIGHT: usize = 160;

/// Bytes por píxel en el framebuffer: un byte para cada canal R, G, B y A.
pub const BYTES_PER_PIXEL: usize = 4;

/// Tamaño total del framebuffer en bytes (240 × 160 × 4 = 153 600).
pub const FRAMEBUFFER_SIZE: usize = SCREEN_WIDTH * SCREEN_HEIGHT * BYTES_PER_PIXEL;

/// Estado completo de una GBA emulada.
///
/// Agrupa la CPU ([`Cpu`]), el bus de memoria ([`Bus`]) y el framebuffer. En
/// fases posteriores sumará la PPU, el scheduler, etc. El frontend interactúa
/// con la emulación únicamente a través de este tipo.
pub struct Gba {
    /// La CPU ARM7TDMI con sus registros, modos y estado.
    cpu: Cpu,

    /// El bus de memoria: ROM del cartucho, RAMs y registros de I/O.
    bus: Bus,

    /// Framebuffer en formato RGBA, con [`FRAMEBUFFER_SIZE`] bytes.
    ///
    /// El orden es fila a fila desde la esquina superior izquierda; cada píxel
    /// son 4 bytes consecutivos: `[R, G, B, A]`.
    framebuffer: Vec<u8>,
}

impl Gba {
    /// Crea una GBA **sin cartucho**: hardware en reset y ROM vacía. Sirve para
    /// la prueba "Hola Ventana" (Fase 1.1) sin necesidad de cargar un juego.
    ///
    /// El framebuffer arranca en un azul sólido de prueba; demuestra que el
    /// núcleo produce píxeles y que el frontend los pinta, y desaparecerá en
    /// cuanto la PPU genere imágenes reales.
    pub fn new() -> Self {
        Self::with_hardware(Cpu::new(), Bus::new(Vec::new()))
    }

    /// Construye la consola a partir de un cartucho ya validado y la deja lista
    /// para ejecutar: vuelca la ROM en el bus y coloca el `PC` en el arranque.
    ///
    /// ## ⚠️ Atajo temporal de desarrollo ("skip BIOS")
    ///
    /// La GBA real arranca en `0x0000_0000` (BIOS), y es la BIOS la que salta al
    /// cartucho. Hasta integrar la BIOS (Mini-Hito 2.3a), apuntamos el `PC`
    /// directamente al inicio de la ROM (`0x0800_0000`) para poder ejecutar ya
    /// el código del juego. **Esa línea desaparece en el 2.3a**, donde el
    /// arranque pasará a ser el real desde la BIOS.
    pub fn with_cartridge(cart: Cartridge) -> Self {
        let mut cpu = Cpu::new();
        cpu.set_pc(bus::ROM_START); // atajo skip-BIOS (temporal, ver 2.3a)
        Self::with_hardware(cpu, Bus::new(cart.into_rom()))
    }

    /// Constructor común: monta la consola con la CPU y el bus dados y pinta el
    /// framebuffer de prueba.
    fn with_hardware(cpu: Cpu, bus: Bus) -> Self {
        let mut gba = Gba {
            cpu,
            bus,
            framebuffer: vec![0; FRAMEBUFFER_SIZE],
        };
        // Azul "GBA" (#1E90FF) como color de prueba visible.
        gba.clear(0x1E, 0x90, 0xFF);
        gba
    }

    /// **Fetch** (Mini-Hito 2.1b): lee —sin ejecutar ni avanzar el `PC`— la
    /// instrucción de 32 bits a la que apunta el Program Counter. Fachada del
    /// frontend sobre [`Cpu::fetch`].
    pub fn fetch(&self) -> u32 {
        self.cpu.fetch(&self.bus)
    }

    /// **Decode** del modo ARM (Mini-Hito 2.1c): identifica el tipo de la
    /// instrucción `instr` aplicando el flujo de dos pasos (condición → opcode).
    /// No ejecuta nada todavía. Fachada del frontend sobre [`Cpu::decode_arm`].
    pub fn decode_arm(&self, instr: u32) -> Decoded {
        self.cpu.decode_arm(instr)
    }

    /// **Decode** del modo THUMB (Mini-Hito 2.1c-bis): identifica el formato de
    /// la instrucción de 16 bits `instr` con un decoder **separado** del de ARM.
    /// No ejecuta nada todavía. Fachada del frontend sobre [`Cpu::decode_thumb`].
    pub fn decode_thumb(&self, instr: u16) -> ThumbInstruction {
        self.cpu.decode_thumb(instr)
    }

    /// El Program Counter actual (`r15`).
    pub fn pc(&self) -> u32 {
        self.cpu.pc()
    }

    /// Rellena todo el framebuffer con un color sólido opaco.
    ///
    /// El canal alfa se fija siempre a `0xFF` (totalmente opaco).
    pub fn clear(&mut self, r: u8, g: u8, b: u8) {
        for pixel in self.framebuffer.chunks_exact_mut(BYTES_PER_PIXEL) {
            pixel[0] = r;
            pixel[1] = g;
            pixel[2] = b;
            pixel[3] = 0xFF;
        }
    }

    /// Devuelve el framebuffer crudo en formato RGBA para que el frontend lo
    /// pinte. Esta es la **única** salida visual del núcleo.
    pub fn framebuffer(&self) -> &[u8] {
        &self.framebuffer
    }
}

impl Default for Gba {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn las_dimensiones_son_las_de_la_gba_real() {
        assert_eq!(SCREEN_WIDTH, 240);
        assert_eq!(SCREEN_HEIGHT, 160);
        assert_eq!(FRAMEBUFFER_SIZE, 240 * 160 * 4);
    }

    #[test]
    fn el_framebuffer_tiene_el_tamano_correcto() {
        let gba = Gba::new();
        assert_eq!(gba.framebuffer().len(), FRAMEBUFFER_SIZE);
    }

    #[test]
    fn clear_rellena_todos_los_pixeles_con_el_color() {
        let mut gba = Gba::new();
        gba.clear(10, 20, 30);
        let fb = gba.framebuffer();

        // Primer píxel.
        assert_eq!(&fb[0..4], &[10, 20, 30, 0xFF]);
        // Último píxel: confirma que el bucle cubre todo el buffer.
        let n = fb.len();
        assert_eq!(&fb[n - 4..n], &[10, 20, 30, 0xFF]);
    }

    #[test]
    fn fetch_lee_la_primera_instruccion_del_cartucho() {
        // Cartucho mínimo con la instrucción 0xEA00002E al inicio de la ROM
        // (en little-endian: [0x2E, 0x00, 0x00, 0xEA]).
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        rom[..4].copy_from_slice(&[0x2E, 0x00, 0x00, 0xEA]);
        let cart = Cartridge::from_bytes(rom).expect("ROM mínima válida");

        let gba = Gba::with_cartridge(cart);

        // El PC arranca apuntando a la ROM (atajo skip-BIOS).
        assert_eq!(gba.pc(), crate::bus::ROM_START);
        // Y el fetch devuelve la instrucción reconstruida en little-endian.
        assert_eq!(gba.fetch(), 0xEA00_002E);
    }

    #[test]
    fn decodifica_la_primera_instruccion_como_salto() {
        // Cartucho cuya primera instrucción es 0xEA00002E (el ejemplo del plan).
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        rom[..4].copy_from_slice(&[0x2E, 0x00, 0x00, 0xEA]);
        let gba = Gba::with_cartridge(Cartridge::from_bytes(rom).unwrap());

        // Fetch → Decode: en reset (CPSR = 0) la condición AL siempre pasa.
        match gba.decode_arm(gba.fetch()) {
            Decoded::Execute(kind) => {
                assert_eq!(kind, ArmInstruction::Branch { link: false });
                // La "Prueba" del Mini-Hito 2.1c.
                assert_eq!(format!("¡Es una instrucción de {kind}!"), "¡Es una instrucción de Salto (B / Branch)!");
            }
            Decoded::ConditionFailed(c) => panic!("no debería fallar la condición: {c:?}"),
        }
    }

    #[test]
    fn decodifica_thumb_con_un_decoder_separado() {
        let gba = Gba::new();
        // 0x2005 = «MOV r0, #5» en THUMB → formato 3 (la "Prueba" de 2.1c-bis).
        assert_eq!(
            gba.decode_thumb(0x2005),
            ThumbInstruction::MoveCompareAddSubImm
        );
    }
}
