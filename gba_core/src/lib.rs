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
//! ## Estado actual (Fase 2.3e — Fase 2.3 completa)
//!
//! Además de cargar y validar el cartucho (Fase 1), el núcleo tiene el
//! esqueleto del hardware: la CPU ARM7TDMI ([`Cpu`]) con sus registros y modos,
//! y el bus de memoria ([`Bus`]) con el mapa de memoria de la consola. La
//! consola [`Gba`] ya integra ambos: [`Gba::with_cartridge`] vuelca la ROM en
//! el bus y coloca el `PC` en el arranque, [`Gba::fetch`] realiza el **"Fetch"**
//! —leer la instrucción a la que apunta el `PC`— y el **"Decode"** identifica el
//! tipo de instrucción: [`Gba::decode_arm`] para el modo ARM (flujo de dos pasos
//! condición → opcode) y [`Gba::decode_thumb`] para el modo THUMB (16 bits, con
//! un decoder separado). El **"Execute"** ya cubre el **procesamiento de datos
//! completo** (Mini-Hito 2.2f): forma inmediata y forma de registro pasada por el
//! *barrel shifter* (`LSL`/`LSR`/`ASR`/`ROR`), incluido el caso `Rd = r15` que
//! convierte la operación en un salto y, con `S=1`, restaura el `CPSR`.
//! Y el **pipeline de 3 etapas** (Mini-Hito 2.1e) ya está modelado: leer `r15`
//! devuelve el `PC` adelantado (+8 en ARM, +4 en THUMB), como el hardware real.
//! Sobre todo eso, el **bucle de ejecución** (Mini-Hito 2.2a) ya encadena
//! fetch→decode→execute paso a paso ([`Gba::run`] / [`Gba::step`]): avanza el
//! `PC` y se detiene limpiamente al llegar a una instrucción todavía no
//! implementada. El **contador de ciclos** (Mini-Hito 2.2c) ya está: cada
//! instrucción ejecutada suma su coste
//! según la región de memoria y si el acceso es secuencial (S) o no (N). Y el
//! **scheduler** (Mini-Hito 2.2d) ya existe como pieza de infraestructura: una
//! cola de eventos ordenada por ciclo ([`Scheduler`]) que será la base de la
//! sincronización de timers/PPU y del Lockstep de la Fase 4; todavía no se
//! integra en el bucle porque aún no hay eventos reales que disparar.
//! Los **saltos** `B`/`BL`/`BX` (Mini-Hito 2.2e) ya se ejecutan: la CPU recorre
//! el código en vez de pararse en el primer salto, y `BX` puede pasar a estado
//! THUMB (cuya ejecución llega después). La **transferencia de PSR** `MRS`/`MSR`
//! (Mini-Hito 2.2g) ya permite leer y escribir el `CPSR`/`SPSR` desde el código,
//! respetando las máscaras de campos y que en modo User solo se toquen los flags
//! —es el prerrequisito de las rutinas que cambian de modo y de la entrada/salida
//! de IRQ y `SWI`—. La **multiplicación** `MUL`/`MLA`/`UMULL`/`UMLAL`/`SMULL`/
//! `SMLAL` (Mini-Hito 2.2h) ya calcula productos de 32 y 64 bits con sus flags y
//! su coste en ciclos variable. Y la **carga/almacén** `LDR`/`STR`/`LDRB`/`STRB`
//! más la de media palabra/byte con signo `LDRH`/`STRH`/`LDRSB`/`LDRSH` (Mini-Hito
//! 2.2i) ya mueve datos entre registros y memoria, con todos sus modos de
//! direccionamiento (offset inmediato/registro, pre/post-indexado, write-back) y
//! las rotaciones del acceso desalineado; por eso el bus se presta ahora como
//! `&mut` al ejecutar. Con eso, el **set ARM** quedó completo —bloque `LDM`/`STM`
//! (2.2j), `SWP`/`SWPB` (2.2k) y las excepciones `SWI`/indefinida (2.2l)— y se
//! sumó el **set THUMB** entero (2.2m). El Mini-Hito **2.3a** cierra por fin el
//! atajo "skip BIOS": si se aporta un `gba_bios.bin` válido (la **BIOS real**,
//! [`Bios`], opcional porque es propietaria), [`Gba::with_cartridge_and_bios`] la
//! carga en `0x0` y arranca el `PC` ahí como el hardware; si no, se mantiene el
//! atajo [`Gba::with_cartridge`]. Y el `fetch` ([`Cpu::fetch`]) ya lee 2 bytes en
//! estado THUMB y 4 en ARM. El Mini-Hito **2.3a-bis** completa ese camino sin
//! BIOS con el **HLE** de los `SWI` (módulo interno `bios_hle`): cuando no hay
//! BIOS real ([`Bus::has_bios`] es `false`), el `SWI` se intercepta y se ejecuta en Rust la
//! función equivalente (división, `CpuSet`, descompresión, matrices afines...) en
//! vez de descarrilar en el vector `0x08`, de modo que el emulador funciona **sin
//! requerir `gba_bios.bin`**. El Mini-Hito **2.3b** añade el **DMA** (módulo
//! [`dma`]): los cuatro canales DMA0–DMA3 con **copia inmediata**. El bus enruta a
//! ellos el bloque de registros de I/O del DMA y, al activarse el `enable` de un
//! canal en modo inmediato, ejecuta la transferencia (los modos por evento
//! —V-Blank/H-Blank/FIFO— quedan armados a la espera de la PPU/APU). El Mini-Hito
//! **2.3c** añade el **sistema de interrupciones** (módulo [`interrupt`]):
//! `IE`/`IF`/`IME` en el bus, una API para que los componentes soliciten IRQs
//! ([`Bus::request_interrupt`], ya conectada al "IRQ al terminar" del DMA) y la
//! **entrada a la excepción de IRQ** en la CPU (vector `0x18`, modo IRQ) cuando hay
//! una pendiente y habilitada. Con ello, el `SWI Halt` deja de ser stub: la CPU
//! puede dormir hasta que llegue una interrupción. El Mini-Hito **2.3d** suma los
//! registros del **SIO** / Cable Link (módulo [`sio`]): `SIODATA`/`SIOCNT`/`RCNT`
//! se almacenan en el bus —**sin** lógica de transferencia, que es de la Fase 4—,
//! para que los juegos puedan configurarlos. El Mini-Hito **2.3e** cierra la Fase
//! 2.3 con los **timers** (módulo [`timers`]) y, al hacerlo, **integra por fin el
//! [`Scheduler`] en el bucle**: los timers programan sus desbordes en la cola de
//! eventos y [`Bus::sync_to_cycle`], llamado tras cada instrucción, los dispara
//! (recarga, IRQ de overflow, cascada). El `Halt` aprovecha el scheduler para
//! **saltar el tiempo muerto** hasta el evento que despierte a la CPU. La frontera
//! con el frontend —entregar un buffer RGBA— no cambia.

pub mod arm;
pub mod bios;
pub(crate) mod bios_hle;
pub mod bus;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod header;
pub mod interrupt;
pub mod scheduler;
pub mod sio;
pub mod thumb;
pub mod timers;

pub use arm::{ArmInstruction, Condition, Decoded};
pub use bios::{Bios, BiosError};
pub use bus::{AccessWidth, Bus, Event};
pub use cartridge::{Cartridge, CartridgeError, MAX_ROM_SIZE, MIN_ROM_SIZE};
pub use dma::{Dma, DmaTransfer, DMA_CHANNELS};
pub use interrupt::{Interrupt, InterruptControl};
pub use sio::Sio;
pub use timers::{Timers, NUM_TIMERS};
pub use cpu::{Cpu, CpuMode, Cpsr, Halt, RunReport, RunStop, StepResult};
pub use header::Header;
pub use scheduler::Scheduler;
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
/// Agrupa la CPU ([`Cpu`]), el bus de memoria ([`Bus`]) y el framebuffer. El bus
/// ya alberga el [`Scheduler`] (integrado en el bucle desde el 2.3e, con los timers
/// como primeros eventos); en fases posteriores se sumará la PPU. El frontend
/// interactúa con la emulación únicamente a través de este tipo.
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
    /// ## Atajo "skip BIOS" (fallback cuando no hay `gba_bios.bin`)
    ///
    /// La GBA real arranca en `0x0000_0000` (BIOS), y es la BIOS la que salta al
    /// cartucho. Cuando **no** se dispone de la BIOS real (que es propietaria y no
    /// se distribuye, ver [`Bios`]), este atajo apunta el `PC` directamente al
    /// inicio de la ROM (`0x0800_0000`) y reproduce a mano el **estado post-BIOS**
    /// (modo System y stack pointers montados, ver [`Cpu::skip_bios_init`]) para
    /// poder ejecutar ya el código del juego. Con la BIOS real disponible, usa
    /// [`Gba::with_cartridge_and_bios`], que arranca de forma fiel desde `0x0`
    /// (Mini-Hito 2.3a).
    pub fn with_cartridge(cart: Cartridge) -> Self {
        let mut cpu = Cpu::new();
        cpu.set_pc(bus::ROM_START); // atajo skip-BIOS (sin BIOS real)
        cpu.skip_bios_init(); // estado post-BIOS: modo System + pila montada
        Self::with_hardware(cpu, Bus::new(cart.into_rom()))
    }

    /// Construye la consola con la **BIOS real** cargada en `0x0` y arranca como
    /// el hardware (Mini-Hito 2.3a): vuelca la BIOS en su región, deja el `PC` en
    /// `0x0000_0000` y **no** aplica el atajo "skip BIOS".
    ///
    /// El `PC` no se toca a propósito: [`Cpu::new`] ya deja `r15 = 0` en el estado
    /// de reset exacto (modo Supervisor, ARM, IRQ/FIQ enmascaradas) desde el que
    /// arranca la BIOS. Es la propia BIOS la que inicializa el hardware, monta los
    /// stack pointers y acaba saltando al cartucho (`0x0800_0000`) de forma
    /// natural —cerrando el atajo del Mini-Hito 2.1b—.
    ///
    /// La BIOS es **opcional**: si el frontend no consigue un `gba_bios.bin`
    /// válido, debe usar [`Gba::with_cartridge`] (el atajo) en su lugar.
    pub fn with_cartridge_and_bios(cart: Cartridge, bios: Bios) -> Self {
        let mut bus = Bus::new(cart.into_rom());
        bus.load_bios(bios.as_bytes());
        Self::with_hardware(Cpu::new(), bus)
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

    /// **Bucle de ejecución** (Mini-Hito 2.2a): corre la CPU encadenando
    /// fetch→decode→execute hasta que se detiene —al toparse con una instrucción
    /// aún no implementada— o hasta ejecutar `max_steps` (salvaguarda contra
    /// bucles infinitos mientras faltan instrucciones por implementar). Delega en
    /// [`Cpu::run`] prestándole el bus.
    pub fn run(&mut self, max_steps: u64) -> RunReport {
        self.cpu.run(&mut self.bus, max_steps)
    }

    /// Ejecuta una sola instrucción: un paso del bucle de [`Gba::run`]. Útil para
    /// un frontend que en el futuro quiera intercalar ejecución y pintado.
    pub fn step(&mut self) -> StepResult {
        self.cpu.step(&mut self.bus)
    }

    /// El Program Counter actual (`r15`).
    pub fn pc(&self) -> u32 {
        self.cpu.pc()
    }

    /// Lee un registro visible de la CPU (`0`–`15`). Pensado para depuración y
    /// para el arnés de test del Mini-Hito 2.2b, que lee el veredicto en `r12`.
    /// `r15` viene con el desfase de pipeline aplicado (ver [`Cpu::reg`]).
    pub fn reg(&self, index: usize) -> u32 {
        self.cpu.reg(index)
    }

    /// Ciclos totales que la CPU ha ejecutado desde el arranque (Mini-Hito 2.2c).
    pub fn cycles(&self) -> u64 {
        self.cpu.cycles()
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

    #[test]
    fn run_ejecuta_la_rom_hasta_una_no_implementada() {
        // MOV r0,#1 ; MOV r1,#2 ; CDP (coprocesador: la GBA no lo tiene, no se
        // implementa nunca): el bucle ejecuta los dos MOV y se detiene en la CDP.
        let programa = [0xE3A0_0001u32, 0xE3A0_1002, 0xEE00_0000];
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        for (i, w) in programa.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let cart = Cartridge::from_bytes(rom).unwrap();
        let mut gba = Gba::with_cartridge(cart);

        let report = gba.run(1_000);
        assert_eq!(report.steps, 2, "dos MOV antes de la CDP");
        assert!(matches!(
            report.stop,
            RunStop::Halted(Halt::Unimplemented { .. })
        ));
    }

    /// Escribe una instrucción de 32 bits (little-endian) en `buf` en `offset`.
    fn poner(buf: &mut [u8], offset: usize, instr: u32) {
        buf[offset..offset + 4].copy_from_slice(&instr.to_le_bytes());
    }

    #[test]
    fn con_bios_arranca_en_0x0_y_salta_a_la_rom() {
        // BIOS **sintética** de test (NO la de Nintendo, que es propietaria):
        // 16 KiB con un mini-programa ARM en 0x0 que carga la dirección de la ROM
        // y salta ahí, como hace la BIOS real al ceder el control (Mini-Hito 2.3a).
        let mut bios = vec![0u8; crate::bus::BIOS_SIZE];
        poner(&mut bios, 0x00, 0xE3A0_0408); // MOV r0, #0x0800_0000 (inicio de la ROM)
        poner(&mut bios, 0x04, 0xE12F_FF10); // BX r0 → salta a la ROM en estado ARM
        let bios = Bios::from_bytes(bios).expect("16 KiB es una BIOS válida");

        // ROM: marca su ejecución (r2 = 42) y termina en el bucle «b .».
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        poner(&mut rom, 0x00, 0xE3A0_202A); // MOV r2, #42
        poner(&mut rom, 0x04, 0xEAFF_FFFE); // b . (fin que el bucle detecta)
        let cart = Cartridge::from_bytes(rom).unwrap();

        let mut gba = Gba::with_cartridge_and_bios(cart, bios);

        // El cierre del atajo del 2.1b: el PC arranca en 0x0 (BIOS), no en la ROM.
        assert_eq!(gba.pc(), 0x0000_0000);

        let report = gba.run(100);
        // La BIOS se ejecutó (r0) y, tras su salto, también el cartucho (r2).
        assert_eq!(gba.reg(0), 0x0800_0000, "la BIOS se ejecutó desde 0x0");
        assert_eq!(gba.reg(2), 42, "saltó al cartucho y ejecutó su código");
        assert!(
            matches!(report.stop, RunStop::Halted(Halt::InfiniteLoop { .. })),
            "se detiene en el «b .» final de la ROM"
        );
    }

    #[test]
    fn dma_end_to_end_la_cpu_programa_una_copia_y_se_ejecuta() {
        // Prueba del Mini-Hito 2.3b end-to-end: la CPU ejecuta código ARM real que
        // configura el DMA0 (EWRAM → IWRAM) y, al escribir el control con enable,
        // la copia inmediata se dispara dentro del bus. Luego el propio programa
        // relee el destino en r5 para que el test pueda comprobarlo con `reg`.
        let programa = [
            0xE3A0_1402u32, // MOV r1, #0x02000000   (EWRAM: origen del dato)
            0xE3A0_0042,    // MOV r0, #0x42          (el dato a copiar)
            0xE581_0000,    // STR r0, [r1]           (EWRAM[0] = 0x42)
            0xE3A0_4404,    // MOV r4, #0x04000000
            0xE284_40B0,    // ADD r4, r4, #0xB0      (r4 = 0x040000B0 = DMA0)
            0xE584_1000,    // STR r1, [r4]           (DMA0SAD = 0x02000000)
            0xE3A0_2403,    // MOV r2, #0x03000000   (IWRAM: destino)
            0xE584_2004,    // STR r2, [r4, #4]       (DMA0DAD = 0x03000000)
            0xE3A0_3001,    // MOV r3, #1
            0xE1C4_30B8,    // STRH r3, [r4, #8]      (DMA0CNT_L = 1 unidad)
            0xE3A0_3C84,    // MOV r3, #0x8400        (enable | 32 bits)
            0xE1C4_30BA,    // STRH r3, [r4, #0xA]    (DMA0CNT_H → dispara la copia)
            0xE592_5000,    // LDR r5, [r2]           (r5 = IWRAM[0], el resultado)
            0xEAFF_FFFE,    // b .                    (fin que el bucle detecta)
        ];
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        for (i, w) in programa.iter().enumerate() {
            poner(&mut rom, i * 4, *w);
        }
        let mut gba = Gba::with_cartridge(Cartridge::from_bytes(rom).unwrap());

        let report = gba.run(1_000);
        assert!(
            matches!(report.stop, RunStop::Halted(Halt::InfiniteLoop { .. })),
            "el programa termina en su «b .» final"
        );
        // El DMA copió el 0x42 de EWRAM a IWRAM, y el LDR lo trajo a r5.
        assert_eq!(gba.reg(5), 0x42, "el DMA debe haber copiado el dato");
    }

    #[test]
    fn irq_end_to_end_dma_dispara_una_interrupcion_atendida_por_la_bios() {
        // Prueba end-to-end del Mini-Hito 2.3c (uniéndolo al 2.3b): un DMA con
        // "IRQ al terminar" levanta la interrupción, la CPU salta al vector 0x18, un
        // handler real (en una BIOS sintética) la reconoce y vuelve con
        // `SUBS pc, lr, #4`, y la ejecución continúa de forma natural.
        //
        // BIOS sintética: arranca saltando a la ROM y tiene el handler de IRQ en
        // 0x18 (marca r10, reconoce IF y retorna).
        let mut bios = vec![0u8; crate::bus::BIOS_SIZE];
        poner(&mut bios, 0x00, 0xE3A0_0408); // MOV r0, #0x0800_0000
        poner(&mut bios, 0x04, 0xE12F_FF10); // BX r0 → salta a la ROM (ARM)
        poner(&mut bios, 0x18, 0xE3A0_A0CA); // [IRQ] MOV r10, #0xCA (marca handler)
        poner(&mut bios, 0x1C, 0xE3A0_B404); // MOV r11, #0x0400_0000
        poner(&mut bios, 0x20, 0xE28B_BC02); // ADD r11, r11, #0x200 (r11=0x0400_0200)
        poner(&mut bios, 0x24, 0xE3E0_C000); // MVN r12, #0 (r12 = 0xFFFF_FFFF)
        poner(&mut bios, 0x28, 0xE1CB_C0B2); // STRH r12, [r11, #2] → IF=0xFFFF (ack)
        poner(&mut bios, 0x2C, 0xE25E_F004); // SUBS pc, lr, #4 (retorno de la IRQ)
        let bios = Bios::from_bytes(bios).expect("16 KiB válida");

        // ROM: habilita interrupciones y programa un DMA0 con IRQ al terminar.
        let programa = [
            0xE321_F01Fu32, // MSR cpsr_c, #0x1F    (modo System, I=0: IRQ habilitadas)
            0xE3A0_4404,     // MOV r4, #0x04000000 (base de I/O)
            0xE3A0_0001,     // MOV r0, #1
            0xE584_0208,     // STR r0, [r4, #0x208] (IME = 1)
            0xE284_5C02,     // ADD r5, r4, #0x200   (r5 = 0x04000200)
            0xE3A0_0C01,     // MOV r0, #0x100       (bit 8 = DMA0)
            0xE1C5_00B0,     // STRH r0, [r5]        (IE = DMA0)
            0xE3A0_1402,     // MOV r1, #0x02000000  (EWRAM: origen)
            0xE3A0_0099,     // MOV r0, #0x99        (el dato)
            0xE581_0000,     // STR r0, [r1]         (EWRAM[0] = 0x99)
            0xE584_10B0,     // STR r1, [r4, #0xB0]  (DMA0SAD = EWRAM)
            0xE3A0_2403,     // MOV r2, #0x03000000  (IWRAM: destino)
            0xE584_20B4,     // STR r2, [r4, #0xB4]  (DMA0DAD = IWRAM)
            0xE3A0_3001,     // MOV r3, #1
            0xE1C4_3BB8,     // STRH r3, [r4, #0xB8] (DMA0CNT_L = 1)
            0xE3A0_3CC4,     // MOV r3, #0xC400      (enable | 32 bits | IRQ-al-terminar)
            0xE1C4_3BBA,     // STRH r3, [r4, #0xBA] (DMA0CNT_H → copia + levanta IRQ)
            0xE592_6000,     // LDR r6, [r2]         (r6 = IWRAM[0]; se ejecuta TRAS la IRQ)
            0xEAFF_FFFE,     // b .                  (fin)
        ];
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        for (i, w) in programa.iter().enumerate() {
            poner(&mut rom, i * 4, *w);
        }
        let mut gba = Gba::with_cartridge_and_bios(Cartridge::from_bytes(rom).unwrap(), bios);

        let report = gba.run(1_000);
        assert!(
            matches!(report.stop, RunStop::Halted(Halt::InfiniteLoop { .. })),
            "termina en el «b .» final"
        );
        assert_eq!(gba.reg(10), 0xCA, "el handler de IRQ se ejecutó");
        assert_eq!(gba.reg(6), 0x99, "y la copia del DMA llegó al destino");
    }

    #[test]
    fn timer_end_to_end_despierta_la_cpu_de_un_halt() {
        // Prueba end-to-end del Mini-Hito 2.3e (uniendo 2.3c): la CPU programa un
        // timer con IRQ y hace `Halt`. Al integrar el scheduler en el bucle, este
        // **adelanta el reloj** hasta el desborde del timer, cuya IRQ (`IE & IF`)
        // despierta a la CPU. Sin la integración del scheduler, el `Halt` pararía en
        // seco. Se usa el camino **sin BIOS** para que el `SWI Halt` entre por el HLE
        // (`cpu.halt()`); con `IME = 0`, la IRQ despierta pero no se atiende, así que
        // la CPU reanuda directamente (no hace falta un manejador en el vector 0x18).
        let programa = [
            0xE3A0_4404u32, // MOV r4, #0x04000000  (base de I/O)
            0xE3A0_0008,    // MOV r0, #0x08         (bit 3 = Timer0)
            0xE284_5C02,    // ADD r5, r4, #0x200    (r5 = 0x04000200)
            0xE1C5_00B0,    // STRH r0, [r5]         (IE = Timer0; IME se queda a 0)
            0xE284_6C01,    // ADD r6, r4, #0x100    (r6 = 0x04000100, base de timers)
            0xE3A0_0000,    // MOV r0, #0
            0xE1C6_00B0,    // STRH r0, [r6]         (TM0CNT_L = 0 → desborda en 65536)
            0xE3A0_00C0,    // MOV r0, #0xC0         (enable | IRQ, prescaler ÷1)
            0xE1C6_00B2,    // STRH r0, [r6, #2]     (TM0CNT_H → arranca el timer)
            0xEF02_0000,    // SWI #0x020000         (Halt: la CPU duerme)
            0xE3A0_7077,    // MOV r7, #0x77         (marca: despertó y siguió)
            0xEAFF_FFFE,    // b .                   (fin)
        ];
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        for (i, w) in programa.iter().enumerate() {
            poner(&mut rom, i * 4, *w);
        }
        let mut gba = Gba::with_cartridge(Cartridge::from_bytes(rom).unwrap());

        let report = gba.run(10_000);
        assert!(
            matches!(report.stop, RunStop::Halted(Halt::InfiniteLoop { .. })),
            "termina en el «b .» final, no dormida"
        );
        assert_eq!(gba.reg(7), 0x77, "la CPU despertó del Halt y siguió ejecutando");
        // El despertar prueba que el timer desbordó: el reloj saltó hasta su
        // desborde (~65536 ciclos), muy por encima de las ~pocas decenas que cuesta
        // el puñado de instrucciones del programa.
        assert!(
            gba.cycles() >= 65_536,
            "el bucle saltó el tiempo muerto hasta el desborde (cycles={})",
            gba.cycles()
        );
    }

    #[test]
    fn sin_bios_el_atajo_skip_bios_arranca_en_la_rom() {
        // El mismo cartucho, pero sin BIOS: el atajo "skip BIOS" arranca el PC en
        // la ROM (0x0800_0000), no en 0x0.
        let mut rom = vec![0u8; MIN_ROM_SIZE];
        poner(&mut rom, 0x00, 0xEAFF_FFFE); // b .
        let gba = Gba::with_cartridge(Cartridge::from_bytes(rom).unwrap());
        assert_eq!(gba.pc(), crate::bus::ROM_START);
    }
}
