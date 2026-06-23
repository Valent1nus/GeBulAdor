//! La CPU **ARM7TDMI** de la Game Boy Advance: registros, estado y modos.
//!
//! Este módulo modela el estado de la CPU y, sobre él, el ciclo
//! Fetch→Decode→Execute tal como está implementado hasta el Mini-Hito 2.2g. De
//! momento cubre:
//!
//! - Los **16 registros visibles** `r0`–`r15` (`r13` = SP, `r14` = LR,
//!   `r15` = PC).
//! - El registro de estado **CPSR** (flags de condición + bits de control).
//! - Los **modos de operación** del procesador y, lo más importante, el
//!   *banking* de registros entre modos.
//! - El **Fetch** ([`Cpu::fetch`]), el **Decode** de ARM ([`Cpu::decode_arm`]) y
//!   THUMB ([`Cpu::decode_thumb`]), y el **Execute** del procesamiento de datos
//!   completo ([`Cpu::execute_data_processing`]: operando inmediato y de registro
//!   por el *barrel shifter*, incluido `Rd = r15`, Mini-Hito 2.2f), de la
//!   transferencia de PSR `MRS`/`MSR` ([`Cpu::execute_psr_transfer`], Mini-Hito
//!   2.2g) y de los saltos `B`/`BL`/`BX` (Mini-Hito 2.2e).
//! - El **desfase del pipeline** de 3 etapas: al leer `r15`, una instrucción ve
//!   el `PC` adelantado (+8 en ARM, +4 en THUMB), ver [`Cpu::reg`] y
//!   [`Cpu::pipeline_offset`].
//!
//! ## ⚠️ Por qué tanto cuidado con los registros "banked"
//!
//! El ARM7TDMI no tiene 16 registros y ya está: tiene 37 registros físicos, de
//! los cuales solo 16 son "visibles" en cada instante. Algunos registros tienen
//! **copias separadas por modo** (*banked registers*):
//!
//! - `r13` (SP) y `r14` (LR) tienen una copia distinta para casi cada modo.
//! - El modo **FIQ** además tiene copias propias de `r8`–`r12`.
//! - Cada modo de excepción (FIQ, IRQ, Supervisor, Abort, Undefined) tiene su
//!   propio **SPSR** (una copia guardada del CPSR de cuando saltó la excepción).
//!
//! Si modeláramos la CPU como un simple `[u32; 16]` plano, el Mini-Hito 2.3c
//! (interrupciones) nos obligaría a **rehacer toda la estructura**, porque al
//! entrar en una IRQ el hardware cambia de modo y, con ello, qué `r13`/`r14`
//! están a la vista. Por eso este diseño separa, desde el día 1, los registros
//! **visibles** (`r`) de su **almacén por banco** ([`Cpu::bank_sp`], etc.).
//!
//! La estrategia es la estándar en emuladores: el camino caliente (ejecutar una
//! instrucción) solo toca el array `r` —rapidísimo, un simple índice—, y solo al
//! **cambiar de modo** (algo poco frecuente) hacemos el intercambio de bancos en
//! [`Cpu::set_mode`].

use crate::arm::{self, ArmInstruction, Decoded};
use crate::bus::{AccessWidth, Bus};
use crate::thumb::ThumbInstruction;

/// Número de registros visibles del ARM7TDMI: `r0`–`r15`.
pub const NUM_REGISTERS: usize = 16;

/// Índice del *Stack Pointer* (`r13`).
pub const SP: usize = 13;
/// Índice del *Link Register* (`r14`), donde `BL` deja la dirección de retorno.
pub const LR: usize = 14;
/// Índice del *Program Counter* (`r15`).
pub const PC: usize = 15;

/// Desfase del `PC` por el **pipeline de 3 etapas** en estado **ARM**: una
/// instrucción que lee `r15` ve su dirección + 8, porque el fetch va dos
/// instrucciones de 4 bytes por delante (Mini-Hito 2.1e).
pub const PC_AHEAD_ARM: u32 = 8;

/// Desfase del `PC` por el pipeline en estado **THUMB**: + 4, dos instrucciones
/// de 2 bytes por delante (Mini-Hito 2.1e).
pub const PC_AHEAD_THUMB: u32 = 4;

/// Bits **realmente implementados** de un PSR en el ARM7TDMI: los flags de
/// condición `NZCV` (31-28) y el byte de control `I`/`F`/`T` + modo (7-0). Los
/// bits 27-8 son reservados: se leen como 0 y `MSR` no puede escribirlos. Lo usa
/// `MSR` (Mini-Hito 2.2g) para no dejar basura en bits que el hardware no tiene,
/// de modo que un `MRS` posterior lea exactamente lo que leería la consola real.
const PSR_VALID: u32 = 0xF000_00FF;

/// Número de "bancos" de registros distintos. Cada banco agrupa los modos que
/// comparten el mismo `r13`/`r14`. User y System comparten banco, así que hay 6:
/// `usr` (User+System), `fiq`, `irq`, `svc`, `abt`, `und`.
const NUM_BANKS: usize = 6;

/// Modos de operación del ARM7TDMI.
///
/// El valor numérico de cada variante son los bits `M[4:0]` tal como aparecen en
/// el CPSR (de ahí los discriminantes explícitos y el `#[repr(u8)]`). La GBA, en
/// la práctica, solo usa de forma habitual User/System (código de juego), IRQ
/// (interrupciones de vídeo, timers...) y Supervisor (llamadas a la BIOS por
/// `SWI`); los demás existen por completitud del hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CpuMode {
    /// Modo de usuario: el de menor privilegio, donde corre el juego.
    User = 0x10,
    /// Fast Interrupt. En la GBA está prácticamente sin usar (no hay fuentes de
    /// FIQ cableadas), pero banca `r8`–`r12` además de `r13`/`r14`.
    Fiq = 0x11,
    /// Interrupt Request: el modo al que salta la CPU ante una IRQ (V-Blank,
    /// timers, DMA...).
    Irq = 0x12,
    /// Supervisor: modo de la BIOS, al que se entra por `SWI` (software
    /// interrupt) y en el que arranca el procesador tras un reset.
    Supervisor = 0x13,
    /// Abort: fallos de acceso a memoria (data/prefetch abort).
    Abort = 0x17,
    /// Undefined: ejecución de una instrucción no definida.
    Undefined = 0x1B,
    /// System: mismos registros que User pero con privilegios. Los juegos lo
    /// usan para tener una pila privilegiada sin estar en un modo de excepción.
    System = 0x1F,
}

impl CpuMode {
    /// Interpreta los 5 bits `M[4:0]` de un CPSR como un modo conocido.
    ///
    /// Devuelve `None` si el patrón no corresponde a ninguno de los 7 modos
    /// válidos del ARM7TDMI (algo que solo puede pasar al escribir el CPSR con
    /// `MSR`, en hitos posteriores; lo tratamos en vez de asumir validez).
    pub fn from_bits(bits: u8) -> Option<CpuMode> {
        Some(match bits & 0x1F {
            0x10 => CpuMode::User,
            0x11 => CpuMode::Fiq,
            0x12 => CpuMode::Irq,
            0x13 => CpuMode::Supervisor,
            0x17 => CpuMode::Abort,
            0x1B => CpuMode::Undefined,
            0x1F => CpuMode::System,
            _ => return None,
        })
    }

    /// Los bits `M[4:0]` que representan este modo dentro del CPSR.
    pub fn bits(self) -> u8 {
        self as u8
    }

    /// Banco de `r13`/`r14`/`SPSR` al que pertenece este modo. User y System
    /// comparten banco (índice 0) porque ven exactamente los mismos registros.
    fn bank(self) -> usize {
        match self {
            CpuMode::User | CpuMode::System => 0,
            CpuMode::Fiq => 1,
            CpuMode::Irq => 2,
            CpuMode::Supervisor => 3,
            CpuMode::Abort => 4,
            CpuMode::Undefined => 5,
        }
    }

    /// `true` si el modo tiene `SPSR` propio. User y System **no** lo tienen
    /// (no son modos de excepción: no hay nada que "guardar y restaurar").
    fn has_spsr(self) -> bool {
        !matches!(self, CpuMode::User | CpuMode::System)
    }
}

/// El registro de estado del programa (CPSR/SPSR), modelado como un *newtype*
/// sobre `u32`.
///
/// Guardamos los 32 bits crudos (que es lo que leen/escriben las instrucciones
/// `MRS`/`MSR` del hardware real) y ofrecemos accesores con nombre para no andar
/// con máscaras de bits por todo el código. Layout en el ARM7TDMI (ARMv4T):
///
/// ```text
///  31 30 29 28           7 6 5 4   0
///  [N][Z][C][V]   ...    [I][F][T][ M[4:0] ]
/// ```
///
/// - **N/Z/C/V** (bits 31-28): flags de condición que pone la ALU y que evalúa
///   el *decode* condicional del Mini-Hito 2.1c.
/// - **I** (bit 7): si está a 1, las IRQ están deshabilitadas.
/// - **F** (bit 6): si está a 1, las FIQ están deshabilitadas.
/// - **T** (bit 5): estado de ejecución; 0 = ARM (32 bits), 1 = THUMB (16 bits).
/// - **M[4:0]** (bits 4-0): el modo actual ([`CpuMode`]).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Cpsr(u32);

impl Cpsr {
    const N: u32 = 1 << 31;
    const Z: u32 = 1 << 30;
    const C: u32 = 1 << 29;
    const V: u32 = 1 << 28;
    const I: u32 = 1 << 7;
    const F: u32 = 1 << 6;
    const T: u32 = 1 << 5;
    const MODE_MASK: u32 = 0x1F;

    /// Construye un CPSR a partir de sus 32 bits crudos.
    pub fn from_bits(bits: u32) -> Self {
        Cpsr(bits)
    }

    /// Los 32 bits crudos (lo que vería un `MRS`).
    pub fn bits(self) -> u32 {
        self.0
    }

    /// Pone o limpia un bit concreto según `value`. Centraliza el patrón
    /// "set/clear de un flag" para no repetirlo en cada setter.
    fn set_flag(&mut self, mask: u32, value: bool) {
        if value {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
    }

    /// Flag N (resultado negativo / bit 31 a 1).
    pub fn n(self) -> bool {
        self.0 & Self::N != 0
    }
    /// Flag Z (resultado cero).
    pub fn z(self) -> bool {
        self.0 & Self::Z != 0
    }
    /// Flag C (acarreo / *borrow* invertido).
    pub fn c(self) -> bool {
        self.0 & Self::C != 0
    }
    /// Flag V (desbordamiento con signo).
    pub fn v(self) -> bool {
        self.0 & Self::V != 0
    }

    /// Fija el flag N.
    pub fn set_n(&mut self, value: bool) {
        self.set_flag(Self::N, value);
    }
    /// Fija el flag Z.
    pub fn set_z(&mut self, value: bool) {
        self.set_flag(Self::Z, value);
    }
    /// Fija el flag C.
    pub fn set_c(&mut self, value: bool) {
        self.set_flag(Self::C, value);
    }
    /// Fija el flag V.
    pub fn set_v(&mut self, value: bool) {
        self.set_flag(Self::V, value);
    }

    /// `true` si la CPU está en estado THUMB (bit T).
    pub fn thumb(self) -> bool {
        self.0 & Self::T != 0
    }
    /// Cambia entre estado ARM (`false`) y THUMB (`true`).
    pub fn set_thumb(&mut self, value: bool) {
        self.set_flag(Self::T, value);
    }

    /// `true` si las IRQ están deshabilitadas (bit I a 1).
    pub fn irq_disabled(self) -> bool {
        self.0 & Self::I != 0
    }
    /// Habilita (`false`) o deshabilita (`true`) las IRQ.
    pub fn set_irq_disabled(&mut self, value: bool) {
        self.set_flag(Self::I, value);
    }

    /// `true` si las FIQ están deshabilitadas (bit F a 1).
    pub fn fiq_disabled(self) -> bool {
        self.0 & Self::F != 0
    }
    /// Habilita (`false`) o deshabilita (`true`) las FIQ.
    pub fn set_fiq_disabled(&mut self, value: bool) {
        self.set_flag(Self::F, value);
    }

    /// Los bits `M[4:0]` (el modo) en crudo.
    pub fn mode_bits(self) -> u8 {
        (self.0 & Self::MODE_MASK) as u8
    }

    /// Escribe los bits de modo. **Privado a propósito:** cambiar el modo sin
    /// hacer el intercambio de bancos rompería la coherencia de `r13`/`r14`. El
    /// único camino correcto para cambiar de modo es [`Cpu::set_mode`].
    fn set_mode_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::MODE_MASK) | (bits as u32 & Self::MODE_MASK);
    }
}

// `Debug` manual para que un volcado del CPSR sea legible ("N:0 Z:1 ... mode:Irq")
// en vez de un número hexadecimal que hay que descifrar a mano.
impl std::fmt::Debug for Cpsr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mode = CpuMode::from_bits(self.mode_bits());
        f.debug_struct("Cpsr")
            .field("N", &(self.n() as u8))
            .field("Z", &(self.z() as u8))
            .field("C", &(self.c() as u8))
            .field("V", &(self.v() as u8))
            .field("I", &(self.irq_disabled() as u8))
            .field("F", &(self.fiq_disabled() as u8))
            .field("T", &(self.thumb() as u8))
            .field("mode", &mode)
            .finish()
    }
}

/// El estado completo de la CPU ARM7TDMI.
///
/// Separa los registros **visibles** (`r`, lo que ve la instrucción en curso)
/// del **almacén por banco** de los registros que se intercambian al cambiar de
/// modo. Ver la explicación de *banking* en la cabecera del módulo.
pub struct Cpu {
    /// Los 16 registros visibles `r0`–`r15` en el modo actual.
    r: [u32; NUM_REGISTERS],

    /// Registro de estado actual.
    cpsr: Cpsr,

    /// `r13` (SP) guardado de cada banco. Al cambiar de modo, el SP visible se
    /// guarda aquí en el banco viejo y se carga el del banco nuevo.
    bank_sp: [u32; NUM_BANKS],
    /// `r14` (LR) guardado de cada banco, gestionado igual que [`Cpu::bank_sp`].
    bank_lr: [u32; NUM_BANKS],
    /// `SPSR` de cada banco. El índice 0 (User/System) no se usa: esos modos no
    /// tienen SPSR.
    spsr: [u32; NUM_BANKS],

    /// Copia de `r8`–`r12` para los modos **no-FIQ** (todos comparten una sola).
    /// Solo se usa para guardar/restaurar al cruzar la frontera con FIQ.
    usr_r8_r12: [u32; 5],
    /// Copia de `r8`–`r12` exclusiva del modo **FIQ**.
    fiq_r8_r12: [u32; 5],

    /// Ciclos totales ejecutados desde el reset (Mini-Hito 2.2c).
    cycles: u64,
    /// Dirección desde la que el próximo fetch sería **secuencial** (S). Si el
    /// fetch coincide, el acceso es S; si no, es N (no secuencial). `None` tras
    /// un reset o un salto, donde el primer fetch es siempre N.
    seq_fetch_addr: Option<u32>,
}

/// Resultado de ejecutar **un paso** de la CPU ([`Cpu::step`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepResult {
    /// Se procesó una instrucción (o un NOP por condición fallida); el bucle
    /// puede continuar.
    Stepped,
    /// La CPU se detuvo; el bucle debe terminar. Lleva el motivo.
    Halted(Halt),
}

/// Motivo por el que la CPU detiene el bucle de ejecución.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Halt {
    /// Se alcanzó una instrucción cuya ejecución aún no está implementada. Se
    /// guarda dónde está, sus bits y su categoría decodificada, para saber qué
    /// falta por implementar.
    Unimplemented {
        /// `PC` (crudo) de la instrucción no implementada.
        pc: u32,
        /// Sus 32 bits tal cual se leyeron del bus.
        instr: u32,
        /// La categoría ARM en la que se clasificó.
        kind: ArmInstruction,
    },
    /// La CPU llegó a un **bucle infinito de un salto** (`b .`): un salto cuyo
    /// destino es su propia dirección. No avanza más. Es la señal de "fin" de las
    /// ROMs de test del Mini-Hito 2.2b (que dejan el veredicto en `r12`) y, en
    /// general, que el programa se ha quedado clavado.
    InfiniteLoop {
        /// `PC` (crudo) del salto que se cierra sobre sí mismo.
        pc: u32,
        /// Sus 32 bits tal cual se leyeron del bus.
        instr: u32,
    },
    /// La CPU entró en estado **THUMB** (por un `BX`) pero la ejecución THUMB aún
    /// no está implementada (llega en 2.2m/2.3a). Se detiene en vez de
    /// malinterpretar la memoria como código ARM.
    ThumbNotImplemented {
        /// `PC` (crudo) donde se quedó, ya en estado THUMB.
        pc: u32,
    },
}

/// Informe de una corrida en bucle ([`Cpu::run`] / [`crate::Gba::run`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunReport {
    /// Instrucciones ejecutadas antes de parar (sin contar la que provocó la
    /// parada, que no llega a ejecutarse).
    pub steps: u64,
    /// Ciclos consumidos durante esta corrida (Mini-Hito 2.2c).
    pub cycles: u64,
    /// Por qué se detuvo la corrida.
    pub stop: RunStop,
}

/// Cómo terminó una corrida de [`Cpu::run`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunStop {
    /// La CPU se detuvo por sí sola (instrucción no implementada).
    Halted(Halt),
    /// Se alcanzó el tope de pasos sin que la CPU se detuviera.
    StepLimit,
}

/// Efecto de ejecutar una instrucción (lo decide [`Cpu::try_execute_arm`] y lo
/// consume [`Cpu::step`]): cómo queda el `PC` y cuántos ciclos internos extra
/// consumió, más allá del fetch del opcode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Executed {
    /// Ejecutada; el `PC` avanza a la siguiente instrucción. `extra_cycles` son
    /// los ciclos internos añadidos (p. ej. el I-cycle del shift por registro).
    Continue {
        /// Ciclos internos extra sobre el fetch del opcode.
        extra_cycles: u64,
    },
    /// Ejecutada y el `PC` quedó fijado a un destino (un salto): no se avanza y
    /// el pipeline se vacía (el siguiente fetch es no secuencial).
    Branched {
        /// Ciclos internos extra, como en [`Executed::Continue`].
        extra_cycles: u64,
    },
    /// Esa instrucción —o esa variante— aún no está implementada.
    Unimplemented,
}

impl Cpu {
    /// Crea una CPU en su estado de **reset** del ARM7TDMI: modo Supervisor,
    /// estado ARM, IRQ y FIQ deshabilitadas, y todos los registros a cero.
    ///
    /// (El `PC` de arranque lo coloca quien construye la consola:
    /// [`crate::Gba::with_cartridge`] lo apunta a la ROM como atajo "skip BIOS"
    /// (Mini-Hito 2.1b), y el 2.3a lo cambiará para arrancar de verdad desde la
    /// BIOS en `0x0`. De momento esto es solo un punto de partida coherente.)
    pub fn new() -> Self {
        let mut cpsr = Cpsr::from_bits(0);
        // El procesador real arranca en Supervisor, en ARM, con las
        // interrupciones enmascaradas hasta que la BIOS las configure.
        cpsr.set_mode_bits(CpuMode::Supervisor.bits());
        cpsr.set_irq_disabled(true);
        cpsr.set_fiq_disabled(true);

        // Todo a cero es coherente: el SP/LR visibles (0) son los del banco
        // Supervisor (también 0), así que no hay desincronización inicial.
        Cpu {
            r: [0; NUM_REGISTERS],
            cpsr,
            bank_sp: [0; NUM_BANKS],
            bank_lr: [0; NUM_BANKS],
            spsr: [0; NUM_BANKS],
            usr_r8_r12: [0; 5],
            fiq_r8_r12: [0; 5],
            cycles: 0,
            seq_fetch_addr: None,
        }
    }

    /// Lee un registro visible por índice (`0`–`15`), **con la semántica de
    /// pipeline para `r15`**.
    ///
    /// El índice siempre proviene de un campo de 4 bits de una instrucción ya
    /// decodificada, así que está garantizado en rango; el `debug_assert!` lo
    /// verifica en builds de depuración sin coste en release.
    ///
    /// **Pipeline de 3 etapas (Mini-Hito 2.1e):** leer `r15` como operando NO
    /// devuelve la dirección de la instrucción en curso, sino la del fetch que va
    /// por delante: `PC + 8` en ARM y `PC + 4` en THUMB (ver
    /// [`Cpu::pipeline_offset`]). Cualquier instrucción que use `r15` para
    /// calcular una dirección (saltos relativos, `LDR Rd, [PC, #imm]`...) asume
    /// este desfase. El valor **crudo** (la dirección de fetch real, sin
    /// adelantar) se obtiene con [`Cpu::pc`], que es lo que usa [`Cpu::fetch`].
    pub fn reg(&self, index: usize) -> u32 {
        debug_assert!(index < NUM_REGISTERS, "índice de registro fuera de rango: {index}");
        if index == PC {
            self.r[PC].wrapping_add(self.pipeline_offset())
        } else {
            self.r[index]
        }
    }

    /// Escribe un registro visible por índice (`0`–`15`).
    pub fn set_reg(&mut self, index: usize, value: u32) {
        debug_assert!(index < NUM_REGISTERS, "índice de registro fuera de rango: {index}");
        self.r[index] = value;
    }

    /// El Program Counter **crudo** (`r15` sin el desfase de pipeline): la
    /// dirección de fetch real. Es lo que usa [`Cpu::fetch`]. Para el valor que
    /// ve una instrucción al leer `r15` como operando (adelantado por el
    /// pipeline) usa [`Cpu::reg`]`(PC)`.
    pub fn pc(&self) -> u32 {
        self.r[PC]
    }
    /// Fija el Program Counter (`r15`).
    pub fn set_pc(&mut self, value: u32) {
        self.r[PC] = value;
    }
    /// El Stack Pointer (`r13`) del modo actual.
    pub fn sp(&self) -> u32 {
        self.r[SP]
    }
    /// El Link Register (`r14`) del modo actual.
    pub fn lr(&self) -> u32 {
        self.r[LR]
    }

    /// El desfase que el **pipeline de 3 etapas** añade al `PC` visible según el
    /// estado de ejecución actual: [`PC_AHEAD_ARM`] (8) en ARM y
    /// [`PC_AHEAD_THUMB`] (4) en THUMB.
    ///
    /// El procesador no ejecuta una instrucción de forma aislada: mientras
    /// ejecuta la de la dirección N, ya ha *fetcheado* la N+2. Por eso `r15`,
    /// leído como operando, vale dos instrucciones por delante. Lo consulta
    /// [`Cpu::reg`] al leer `r15`.
    pub fn pipeline_offset(&self) -> u32 {
        if self.cpsr.thumb() {
            PC_AHEAD_THUMB
        } else {
            PC_AHEAD_ARM
        }
    }

    /// **Fetch**: lee la instrucción ARM (32 bits) a la que apunta el `PC`, en
    /// little-endian, a través del bus. Es la primera etapa del ciclo
    /// Fetch→Decode→Execute (Mini-Hito 2.1b).
    ///
    /// No avanza ni modifica el `PC`: es una lectura pura, y usa el `PC` **crudo**
    /// ([`Cpu::pc`]), no el adelantado por el pipeline —el fetch lee justo la
    /// instrucción a la que apunta `PC`—. El avance del puntero llega con el
    /// bucle de ejecución (Mini-Hito 2.2a).
    ///
    /// De momento siempre lee 4 bytes (modo ARM). El Mini-Hito 2.3a añadirá la
    /// rama THUMB: leer 2 bytes cuando el bit `T` del CPSR esté activo.
    pub fn fetch(&self, bus: &Bus) -> u32 {
        bus.read_u32(self.pc())
    }

    /// **Decode** en modo ARM (Mini-Hito 2.1c): clasifica la instrucción `instr`
    /// aplicando el flujo de dos pasos —primero la condición contra el CPSR
    /// actual, y solo si se cumple, el opcode—. No ejecuta nada todavía.
    ///
    /// Es una fachada sobre [`crate::arm::decode`] que le pasa el CPSR vigente.
    pub fn decode_arm(&self, instr: u32) -> Decoded {
        arm::decode(instr, self.cpsr())
    }

    /// **Decode** en modo THUMB (Mini-Hito 2.1c-bis): clasifica la instrucción
    /// de 16 bits `instr`. A diferencia de [`Cpu::decode_arm`], **no** consulta
    /// el CPSR, porque THUMB no lleva condición embebida (de ahí que ignore
    /// `&self`); la única condicional es el salto `B<cond>`. Es una fachada sobre
    /// [`ThumbInstruction::decode`], que vive en un decoder separado del de ARM.
    pub fn decode_thumb(&self, instr: u16) -> ThumbInstruction {
        ThumbInstruction::decode(instr)
    }

    /// Ejecuta una instrucción de **procesamiento de datos** completa (Mini-Hito
    /// 2.2f): calcula el resultado de la ALU (`MOV`, `ADD`, `SUB`, `AND`...), lo
    /// escribe en `Rd` salvo en las comparaciones (`TST`/`TEQ`/`CMP`/`CMN`) y
    /// actualiza los flags `N/Z/C/V` si el bit `S` (20) está activo.
    ///
    /// Cubre las **dos formas** del operando 2:
    /// - **Inmediato** (bit 25 = 1): un valor de 8 bits rotado a la derecha.
    /// - **Registro** (bit 25 = 0): `Rm` pasado por el *barrel shifter*
    ///   (`LSL`/`LSR`/`ASR`/`ROR`), con la cantidad o bien inmediata (bits 11-7)
    ///   o bien en un registro `Rs` (bit 4 = 1). El shifter produce además el
    ///   **carry** que usan las operaciones lógicas; ver [`shift_by_immediate`] y
    ///   [`shift_by_register`].
    ///
    /// Y el caso especial **`Rd = r15`**: la operación se vuelve un salto (devuelve
    /// [`Executed::Branched`], que hace a [`Cpu::step`] vaciar el pipeline); si
    /// además `S = 1`, restaura el `CPSR` desde el `SPSR` del modo actual —el
    /// retorno de excepción, p. ej. `MOVS pc, lr`—, que puede cambiar de modo y de
    /// estado ARM/THUMB.
    ///
    /// Se asume que la condición ya se evaluó (vía [`Cpu::decode_arm`]) y se
    /// cumple. Devuelve el [`Executed`] con los ciclos extra (el I-cycle del shift
    /// por registro) y el efecto sobre el `PC`.
    pub fn execute_data_processing(&mut self, instr: u32) -> Executed {
        // El carry de ENTRADA lo necesitan ADC/SBC/RSC y el carry del shifter:
        // hay que leerlo antes de tocar los flags.
        let carry_in = self.cpsr().c();

        let is_immediate = (instr & (1 << 25)) != 0;
        // Shift cuya cantidad vive en un registro (solo forma de registro, bit 4).
        let register_shift = !is_immediate && (instr & (1 << 4)) != 0;

        // Operando 1 (Rn). ⚠️ Trampa del pipeline: con shift por registro, leer
        // r15 da PC+12 (no +8), por el ciclo interno extra que añade ese shift.
        let rn = ((instr >> 16) & 0xF) as usize;
        let a = self.reg_operand(rn, register_shift);

        // Operando 2 (b) y carry del shifter.
        let (b, shifter_carry) = if is_immediate {
            // Inmediato: 8 bits rotados a la derecha por (bits 11-8)×2.
            let rotate = ((instr >> 8) & 0xF) * 2;
            let value = (instr & 0xFF).rotate_right(rotate);
            // Con rotación 0 el carry se conserva; con rotación, es el bit 31.
            let carry = if rotate == 0 { carry_in } else { bit(value, 31) };
            (value, carry)
        } else {
            let rm = (instr & 0xF) as usize;
            let value = self.reg_operand(rm, register_shift);
            let ty = ShiftType::from_bits(instr >> 5);
            if register_shift {
                // Cantidad = byte bajo de Rs (bits 11-8).
                let rs = ((instr >> 8) & 0xF) as usize;
                let amount = self.reg(rs) & 0xFF;
                shift_by_register(ty, amount, value, carry_in)
            } else {
                // Cantidad inmediata (bits 11-7, 0..=31).
                let amount = (instr >> 7) & 0x1F;
                shift_by_immediate(ty, amount, value, carry_in)
            }
        };

        let opcode = (instr >> 21) & 0xF;
        let sets_flags = (instr & (1 << 20)) != 0;
        let rd = ((instr >> 12) & 0xF) as usize;

        // --- Operación de la ALU ------------------------------------------
        // Las lógicas dejan V sin tocar (`None`) y usan el carry del shifter;
        // las aritméticas obtienen carry/overflow de la suma. La resta se modela
        // como `a + !b + 1`, así que [`alu_add`] cubre todos los casos.
        let (result, carry, overflow): (u32, bool, Option<bool>) = match opcode {
            0x0 => (a & b, shifter_carry, None),     // AND
            0x1 => (a ^ b, shifter_carry, None),     // EOR
            0x2 => with_v(alu_add(a, !b, true)),     // SUB
            0x3 => with_v(alu_add(b, !a, true)),     // RSB
            0x4 => with_v(alu_add(a, b, false)),     // ADD
            0x5 => with_v(alu_add(a, b, carry_in)),  // ADC
            0x6 => with_v(alu_add(a, !b, carry_in)), // SBC
            0x7 => with_v(alu_add(b, !a, carry_in)), // RSC
            0x8 => (a & b, shifter_carry, None),     // TST
            0x9 => (a ^ b, shifter_carry, None),     // TEQ
            0xA => with_v(alu_add(a, !b, true)),     // CMP
            0xB => with_v(alu_add(a, b, false)),     // CMN
            0xC => (a | b, shifter_carry, None),     // ORR
            0xD => (b, shifter_carry, None),         // MOV
            0xE => (a & !b, shifter_carry, None),    // BIC
            0xF => (!b, shifter_carry, None),        // MVN
            _ => unreachable!("opcode = (instr >> 21) & 0xF está en 0..=15"),
        };

        // El shift por registro añade un ciclo interno (I-cycle); ver [`Cpu::step`].
        let extra_cycles = u64::from(register_shift);

        // --- Flags y escritura del resultado ------------------------------
        if matches!(opcode, 0x8..=0xB) {
            // Comparaciones (TST/TEQ/CMP/CMN): siempre fijan flags, nunca escriben
            // Rd y nunca son un salto (el campo Rd se ignora).
            self.write_flags(result, carry, overflow);
            Executed::Continue { extra_cycles }
        } else if rd == PC {
            // Rd = r15: la operación es un salto. Con S=1 es además un retorno de
            // excepción (restaura el CPSR desde el SPSR).
            if sets_flags {
                self.restore_cpsr_from_spsr();
            }
            // Alinea el destino al ancho del estado resultante (THUMB: ½ palabra).
            let target = if self.cpsr().thumb() {
                result & !1
            } else {
                result & !3
            };
            self.set_pc(target);
            Executed::Branched { extra_cycles }
        } else {
            if sets_flags {
                self.write_flags(result, carry, overflow);
            }
            self.set_reg(rd, result);
            Executed::Continue { extra_cycles }
        }
    }

    /// Lee un registro como **operando** de una instrucción, con la trampa del
    /// pipeline para `r15` cuando la cantidad de shift vive en un registro: en ese
    /// caso el `PC` visible va +12 (no +8), por el ciclo interno extra que añade
    /// el shift por registro. Para el resto de casos equivale a [`Cpu::reg`].
    fn reg_operand(&self, index: usize, register_shift: bool) -> u32 {
        let base = self.reg(index);
        if register_shift && index == PC {
            base.wrapping_add(4)
        } else {
            base
        }
    }

    /// Vuelca el resultado de la ALU en los flags `N/Z/C/V`. Un `overflow == None`
    /// (operaciones lógicas) deja `V` intacto.
    fn write_flags(&mut self, result: u32, carry: bool, overflow: Option<bool>) {
        let cpsr = self.cpsr_mut();
        cpsr.set_n(bit(result, 31));
        cpsr.set_z(result == 0);
        cpsr.set_c(carry);
        if let Some(v) = overflow {
            cpsr.set_v(v);
        }
    }

    /// Restaura el `CPSR` desde el `SPSR` del modo actual: el **retorno de
    /// excepción** que dispara un data-processing con `Rd = r15` y `S = 1`. Como
    /// el `SPSR` lleva sus propios bits de modo, esto puede cambiar de modo —con
    /// el intercambio de bancos de [`Cpu::set_mode`]— y de estado ARM/THUMB.
    ///
    /// En User/System no hay `SPSR` (caso indefinido del hardware): no se toca el
    /// `CPSR`.
    fn restore_cpsr_from_spsr(&mut self) {
        let spsr = match self.spsr() {
            Some(spsr) => spsr,
            None => return,
        };
        match CpuMode::from_bits((spsr & 0x1F) as u8) {
            Some(new_mode) => {
                self.set_mode(new_mode); // intercambia los bancos al nuevo modo
                self.cpsr = Cpsr::from_bits(spsr); // y restaura el CPSR completo
            }
            None => debug_assert!(false, "SPSR con bits de modo inválidos al restaurar"),
        }
    }

    // ===== Transferencia de PSR: MRS / MSR (Mini-Hito 2.2g) =================

    /// Ejecuta una **transferencia con el registro de estado** (Mini-Hito 2.2g):
    /// leer el `CPSR`/`SPSR` a un registro (`MRS`) o escribirlo desde un registro
    /// o un inmediato (`MSR`).
    ///
    /// El bit 22 (`R`) elige entre `CPSR` (0) y `SPSR` (1); el bit 21 distingue
    /// `MRS` (0) de `MSR` (1) —el decode ya garantizó que esto es PSR-transfer y
    /// no un `BX` ni una comparación—. Ninguna de las dos salta: devuelven
    /// [`Executed::Continue`] (cuestan 1S, ya contabilizado por el fetch).
    pub fn execute_psr_transfer(&mut self, instr: u32) -> Executed {
        let use_spsr = (instr & (1 << 22)) != 0;
        let is_msr = (instr & (1 << 21)) != 0;
        if is_msr {
            self.execute_msr(instr, use_spsr);
        } else {
            self.execute_mrs(instr, use_spsr);
        }
        Executed::Continue { extra_cycles: 0 }
    }

    /// `MRS Rd, <PSR>`: copia el `CPSR` (o el `SPSR` del modo actual) a `Rd`.
    ///
    /// En User/System no existe `SPSR`; leerlo es impredecible en el hardware, así
    /// que como salvaguarda devolvemos el `CPSR` en vez de un valor basura.
    fn execute_mrs(&mut self, instr: u32, from_spsr: bool) {
        let rd = ((instr >> 12) & 0xF) as usize;
        let value = if from_spsr {
            self.spsr().unwrap_or_else(|| self.cpsr.bits())
        } else {
            self.cpsr.bits()
        };
        self.set_reg(rd, value);
    }

    /// `MSR <PSR>_<campos>, Rm`/`#imm`: escribe el `CPSR`/`SPSR` respetando la
    /// **máscara de campos** del encoding (bits 19-16, un bit por byte del PSR).
    ///
    /// El operando fuente es un registro `Rm` o un inmediato de 8 bits rotado
    /// (bit 25 = 1), exactamente como en el procesamiento de datos. En modo
    /// **User** solo se permiten los flags de condición: los bits de control
    /// (modo, `I`, `F`, `T`) son de solo lectura ahí.
    fn execute_msr(&mut self, instr: u32, to_spsr: bool) {
        // Operando fuente: inmediato rotado (bit 25 = 1) o registro `Rm`.
        let value = if (instr & (1 << 25)) != 0 {
            let rotate = ((instr >> 8) & 0xF) * 2;
            (instr & 0xFF).rotate_right(rotate)
        } else {
            self.reg((instr & 0xF) as usize)
        };

        // Máscara de campos (bits 19-16): cada bit habilita un byte del PSR.
        let fields = (instr >> 16) & 0xF;
        let mut mask = 0;
        if fields & 0b0001 != 0 {
            mask |= 0x0000_00FF; // c: byte de control (modo, I, F, T)
        }
        if fields & 0b0010 != 0 {
            mask |= 0x0000_FF00; // x: extensión (reservado en ARMv4T)
        }
        if fields & 0b0100 != 0 {
            mask |= 0x00FF_0000; // s: estado (reservado en ARMv4T)
        }
        if fields & 0b1000 != 0 {
            mask |= 0xFF00_0000; // f: byte de flags (NZCV)
        }
        // Recorta a los bits que el ARM7TDMI implementa de verdad (el resto es
        // reservado y no se puede escribir).
        mask &= PSR_VALID;

        if to_spsr {
            self.write_spsr_masked(value, mask);
        } else {
            // En modo User solo se pueden cambiar los flags, nunca los de control.
            let mask = if self.mode() == CpuMode::User {
                mask & 0xF000_0000
            } else {
                mask
            };
            self.write_cpsr_masked(value, mask);
        }
    }

    /// Escribe el `CPSR` aplicando `mask` sobre `value`. Si la escritura cambia
    /// los **bits de modo**, el cambio se enruta por [`Cpu::set_mode`] para
    /// intercambiar los bancos de `r13`/`r14` (y de `r8`–`r12` al cruzar FIQ);
    /// hacerlo "a mano" dejaría los registros visibles incoherentes con el modo.
    fn write_cpsr_masked(&mut self, value: u32, mask: u32) {
        let old = self.cpsr.bits();
        let new = (old & !mask) | (value & mask);
        if (new ^ old) & Cpsr::MODE_MASK != 0 {
            match CpuMode::from_bits(new as u8) {
                // set_mode intercambia bancos y fija los bits de modo; después
                // aplicamos el resto del PSR (flags, I/F/T).
                Some(new_mode) => {
                    self.set_mode(new_mode);
                    self.cpsr = Cpsr::from_bits(new);
                }
                // Modo inválido (impredecible en hardware): conservamos el modo
                // actual y aplicamos solo los demás bits, sin corromper los bancos.
                None => {
                    debug_assert!(false, "MSR escribió bits de modo inválidos");
                    self.cpsr =
                        Cpsr::from_bits((new & !Cpsr::MODE_MASK) | (old & Cpsr::MODE_MASK));
                }
            }
        } else {
            self.cpsr = Cpsr::from_bits(new);
        }
    }

    /// Escribe el `SPSR` del modo actual aplicando `mask`. A diferencia del
    /// `CPSR`, no tiene restricción de privilegio ni cambia bancos. En User/System
    /// no hay `SPSR`: la escritura se descarta, como en el hardware.
    fn write_spsr_masked(&mut self, value: u32, mask: u32) {
        if let Some(old) = self.spsr() {
            self.set_spsr((old & !mask) | (value & mask));
        }
    }

    // ===== Multiplicación: MUL / MLA / largas (Mini-Hito 2.2h) ==============

    /// Ejecuta una **multiplicación de 32 bits** (Mini-Hito 2.2h): `MUL` o, con el
    /// bit `A` (21) activo, `MLA` (multiplica y acumula).
    ///
    /// Encoding: `cccc 0000 00AS dddd nnnn ssss 1001 mmmm`. Calcula
    /// `Rd = Rm·Rs` (`+ Rn` si es `MLA`) quedándose con los **32 bits bajos** del
    /// producto —que son idénticos con y sin signo, de ahí que una sola operación
    /// sirva para ambas interpretaciones—.
    ///
    /// Con `S = 1` actualiza `N` (bit 31 del resultado) y `Z` (resultado nulo). El
    /// flag `C` queda **UNPREDECIBLE** en el ARM7TDMI real tras un *multiply* (su
    /// valor depende del estado interno del multiplicador de Booth); aquí lo
    /// **preservamos** —convención de emulador que respetan las gba-tests, que
    /// enmascaran `C` en esta familia— y `V` no se toca.
    ///
    /// Nunca escribe `r15`, así que siempre continúa ([`Executed::Continue`]). El
    /// coste es `1S + mI` (`MUL`) o `1S + (m+1)I` (`MLA`): la `S` la cuenta el fetch
    /// del bucle y los I-cycles, variables según `Rs`, van como `extra_cycles`
    /// (ver [`multiply_internal_cycles`]).
    ///
    /// Usar `r15` como cualquier operando es UNPREDECIBLE en ARMv4 y no aparece en
    /// código real; no se le da un trato especial (leerlo daría `PC + 8`).
    pub fn execute_multiply(&mut self, instr: u32) -> Executed {
        let accumulate = (instr & (1 << 21)) != 0;
        let sets_flags = (instr & (1 << 20)) != 0;
        let rd = ((instr >> 16) & 0xF) as usize;
        let rn = ((instr >> 12) & 0xF) as usize;
        let rs = ((instr >> 8) & 0xF) as usize;
        let rm = (instr & 0xF) as usize;

        // `Rm` es el operando; `Rs` el multiplicador, del que depende el coste.
        let multiplier = self.reg(rs);
        let mut result = self.reg(rm).wrapping_mul(multiplier);
        if accumulate {
            result = result.wrapping_add(self.reg(rn));
        }
        self.set_reg(rd, result);

        if sets_flags {
            let cpsr = self.cpsr_mut();
            cpsr.set_n(bit(result, 31));
            cpsr.set_z(result == 0);
            // C queda UNPREDECIBLE tras multiplicar en el ARM7TDMI: lo preservamos.
            // V no se modifica.
        }

        // 1S + mI (MUL) o 1S + (m+1)I (MLA). La terminación temprana del Booth usa
        // el criterio con signo (todo ceros o todo unos) para MUL/MLA.
        let m = multiply_internal_cycles(multiplier, true);
        Executed::Continue {
            extra_cycles: if accumulate { m + 1 } else { m },
        }
    }

    /// Ejecuta una **multiplicación larga de 64 bits** (Mini-Hito 2.2h):
    /// `UMULL`/`UMLAL` (sin signo) y `SMULL`/`SMLAL` (con signo).
    ///
    /// Encoding: `cccc 0000 1UAS hhhh llll ssss 1001 mmmm`, donde el bit `U` (22)
    /// elige sin signo (0) o con signo (1) y el bit `A` (21) activa la acumulación.
    /// El producto de 64 bits se reparte entre `RdHi` (bits 19-16, palabra alta) y
    /// `RdLo` (bits 15-12, palabra baja); en las variantes con acumulación se le
    /// **suma** el valor previo de `RdHi:RdLo` (módulo 2⁶⁴).
    ///
    /// Con `S = 1` fija `N` (bit 63 del resultado) y `Z` (los 64 bits a cero); `C` y
    /// `V` quedan UNPREDECIBLES en el hardware, así que se preservan (igual que en
    /// [`Cpu::execute_multiply`]). Nunca salta: devuelve [`Executed::Continue`].
    ///
    /// Coste: `1S + (m+1)I` (sin acumular) o `1S + (m+2)I` (acumulando). La
    /// terminación temprana por «todo unos» **solo** aplica a las versiones con
    /// signo; las sin signo solo cuentan los bits altos a cero (ver
    /// [`multiply_internal_cycles`]).
    pub fn execute_multiply_long(&mut self, instr: u32) -> Executed {
        let signed = (instr & (1 << 22)) != 0;
        let accumulate = (instr & (1 << 21)) != 0;
        let sets_flags = (instr & (1 << 20)) != 0;
        let rd_hi = ((instr >> 16) & 0xF) as usize;
        let rd_lo = ((instr >> 12) & 0xF) as usize;
        let rs = ((instr >> 8) & 0xF) as usize;
        let rm = (instr & 0xF) as usize;

        let multiplier = self.reg(rs);
        let operand = self.reg(rm);

        // Producto de 64 bits según el bit U: con signo extiende ambos operandos a
        // i64 (y reinterpreta a u64); sin signo los amplía a u64 directamente.
        let mut product: u64 = if signed {
            (i64::from(operand as i32).wrapping_mul(i64::from(multiplier as i32))) as u64
        } else {
            u64::from(operand).wrapping_mul(u64::from(multiplier))
        };

        if accumulate {
            // Acumulador previo = RdHi:RdLo (RdHi es la palabra alta).
            let acc = (u64::from(self.reg(rd_hi)) << 32) | u64::from(self.reg(rd_lo));
            product = product.wrapping_add(acc);
        }

        self.set_reg(rd_lo, product as u32);
        self.set_reg(rd_hi, (product >> 32) as u32);

        if sets_flags {
            let cpsr = self.cpsr_mut();
            cpsr.set_n(product >> 63 != 0); // bit 63 = signo del resultado de 64 bits
            cpsr.set_z(product == 0); // nulo solo si AMBAS palabras son cero
            // C y V quedan UNPREDECIBLES en el hardware: se preservan.
        }

        // 1S + (m+1)I (largo) o 1S + (m+2)I (largo con acumulación).
        let m = multiply_internal_cycles(multiplier, signed);
        Executed::Continue {
            extra_cycles: if accumulate { m + 2 } else { m + 1 },
        }
    }

    // ===== Bucle de ejecución (Mini-Hito 2.2a) ==============================

    /// El tamaño en bytes de la instrucción actual según el estado: 4 en ARM, 2
    /// en THUMB. Es cuánto avanza el `PC` hacia la siguiente instrucción.
    pub fn instruction_size(&self) -> u32 {
        if self.cpsr.thumb() {
            2
        } else {
            4
        }
    }

    /// Avanza el `PC` a la siguiente instrucción. Lo usa [`Cpu::step`] tras una
    /// instrucción que **no** sea un salto (los saltos fijan el `PC` ellos
    /// mismos). Opera sobre el `PC` crudo ([`Cpu::pc`]).
    fn advance_pc(&mut self) {
        self.set_pc(self.pc().wrapping_add(self.instruction_size()));
    }

    /// Ciclos totales que la CPU ha ejecutado desde el reset (Mini-Hito 2.2c).
    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    /// Ejecuta **un paso**: fetch → decode → execute de una sola instrucción
    /// (Mini-Hito 2.2a), sumando sus ciclos (2.2c) y avanzando o saltando el `PC`.
    ///
    /// Solo se ejecuta el set **ARM** (el fetch THUMB de 2 bytes llega en el
    /// 2.3a). De ARM están implementados el **procesamiento de datos completo**
    /// (Mini-Hito 2.2f: inmediato y registro por el barrel shifter, incluido
    /// `Rd = r15`), la **transferencia de PSR** `MRS`/`MSR` (Mini-Hito 2.2g), la
    /// **multiplicación** `MUL`/`MLA`/largas (Mini-Hito 2.2h) y los **saltos**
    /// `B`/`BL`/`BX` (Mini-Hito 2.2e); ante cualquier otra instrucción la CPU se
    /// detiene con [`StepResult::Halted`], **sin** avanzar el `PC` (queda en la
    /// instrucción culpable, para inspeccionarla).
    pub fn step(&mut self, bus: &Bus) -> StepResult {
        let pc = self.pc();

        // Tras un `BX` a THUMB el estado cambia, pero la ejecución THUMB aún no
        // está (2.2m/2.3a): paramos limpiamente en vez de malinterpretar bytes.
        if self.cpsr.thumb() {
            return StepResult::Halted(Halt::ThumbNotImplemented { pc });
        }

        let instr = self.fetch(bus);

        // Coste del fetch del opcode (32 bits en ARM): N o S según haya sido
        // secuencial respecto al fetch anterior (Mini-Hito 2.2c).
        let seq = self.seq_fetch_addr == Some(pc);
        let fetch_cycles = bus.access_cycles(pc, AccessWidth::Word, seq) as u64;

        match self.decode_arm(instr) {
            // Condición no cumplida: la instrucción es un NOP de un ciclo. Lo
            // único que hace es dejar pasar el tiempo, así que solo avanzamos.
            Decoded::ConditionFailed(_) => {
                self.advance_pc();
                self.account_step(fetch_cycles);
                StepResult::Stepped
            }
            // Un «b .» (salto a su propia dirección) es un bucle infinito: lo
            // reconocemos sin ejecutarlo (colgaría) — es la señal de "fin" de las
            // ROMs de test (2.2b).
            Decoded::Execute(kind) if is_branch_to_self(kind, instr, pc) => {
                StepResult::Halted(Halt::InfiniteLoop { pc, instr })
            }
            Decoded::Execute(kind) => match self.try_execute_arm(kind, instr) {
                Executed::Continue { extra_cycles } => {
                    self.advance_pc();
                    self.account_step(fetch_cycles + extra_cycles);
                    StepResult::Stepped
                }
                Executed::Branched { extra_cycles } => {
                    // El salto ya fijó el `PC`. Coste del ARM7TDMI = 2S + 1N: el
                    // fetch del salto, el prefetch secuencial descartado y el
                    // fetch del destino (que cuenta el próximo paso como N, por el
                    // flush). Vaciamos el pipeline → el siguiente fetch es N. Un
                    // shift por registro que escribe el PC suma además su I-cycle.
                    let discarded =
                        bus.access_cycles(pc.wrapping_add(4), AccessWidth::Word, true) as u64;
                    self.cycles += fetch_cycles + discarded + extra_cycles;
                    self.seq_fetch_addr = None;
                    StepResult::Stepped
                }
                Executed::Unimplemented => {
                    StepResult::Halted(Halt::Unimplemented { pc, instr, kind })
                }
            },
        }
    }

    /// Contabiliza un paso ejecutado: suma sus `cycles` al total y anota desde
    /// dónde sería secuencial el siguiente fetch (para distinguir accesos S de N).
    fn account_step(&mut self, cycles: u64) {
        self.cycles += cycles;
        self.seq_fetch_addr = Some(self.pc());
    }

    /// Ejecuta pasos en bucle hasta que la CPU se detiene ([`StepResult::Halted`])
    /// o hasta completar `max_steps` instrucciones (Mini-Hito 2.2a).
    ///
    /// El tope `max_steps` es una **salvaguarda**: mientras falten instrucciones
    /// por implementar, una secuencia de NOPs (p. ej. memoria a cero) avanzaría
    /// el `PC` indefinidamente; sin un límite, el bucle no terminaría nunca.
    pub fn run(&mut self, bus: &Bus, max_steps: u64) -> RunReport {
        let cycles_start = self.cycles;
        let mut steps = 0;
        while steps < max_steps {
            match self.step(bus) {
                StepResult::Stepped => steps += 1,
                StepResult::Halted(halt) => {
                    return RunReport {
                        steps,
                        cycles: self.cycles - cycles_start,
                        stop: RunStop::Halted(halt),
                    };
                }
            }
        }
        RunReport {
            steps,
            cycles: self.cycles - cycles_start,
            stop: RunStop::StepLimit,
        }
    }

    /// Intenta ejecutar la instrucción ARM `kind` (bits crudos en `instr`),
    /// asumiendo que su condición ya pasó. Devuelve cómo afecta al `PC`
    /// ([`Executed::Continue`] / [`Executed::Branched`]) o
    /// [`Executed::Unimplemented`] si esa instrucción o variante aún no existe.
    ///
    /// A medida que se implementen instrucciones, este `match` ganará ramas.
    fn try_execute_arm(&mut self, kind: ArmInstruction, instr: u32) -> Executed {
        match kind {
            // Procesamiento de datos completo (Mini-Hito 2.2f): ambas formas del
            // operando 2 (inmediato y registro por el barrel shifter) y el caso
            // `Rd = r15`. La propia ejecución decide si fue un salto y sus ciclos
            // extra. Ver [`Cpu::execute_data_processing`].
            ArmInstruction::DataProcessing => self.execute_data_processing(instr),
            // Transferencia de PSR `MRS`/`MSR` (Mini-Hito 2.2g): leer/escribir el
            // CPSR/SPSR. No es un salto. Ver [`Cpu::execute_psr_transfer`].
            ArmInstruction::PsrTransfer => self.execute_psr_transfer(instr),
            // Multiplicación de 32 bits `MUL`/`MLA` (Mini-Hito 2.2h).
            ArmInstruction::Multiply => self.execute_multiply(instr),
            // Multiplicación larga de 64 bits `UMULL`/`UMLAL`/`SMULL`/`SMLAL`
            // (Mini-Hito 2.2h). Ver [`Cpu::execute_multiply_long`].
            ArmInstruction::MultiplyLong => self.execute_multiply_long(instr),
            // Saltos relativos `B`/`BL` (Mini-Hito 2.2e).
            ArmInstruction::Branch { link } => {
                self.execute_branch(instr, link);
                Executed::Branched { extra_cycles: 0 }
            }
            // `BX`: salto a registro con posible cambio de estado ARM/THUMB.
            ArmInstruction::BranchExchange => {
                self.execute_bx(instr);
                Executed::Branched { extra_cycles: 0 }
            }
            _ => Executed::Unimplemented,
        }
    }

    /// Ejecuta un salto relativo `B` (o `BL` si `link`): Mini-Hito 2.2e. El
    /// destino se calcula sobre el `PC` adelantado por el pipeline (`pc + 8`, ver
    /// 2.1e) más el desplazamiento de 24 bits con signo (×4). `BL` guarda en `LR`
    /// la dirección de la instrucción siguiente al salto.
    fn execute_branch(&mut self, instr: u32, link: bool) {
        if link {
            // Retorno = la instrucción siguiente al `BL` (PC crudo + su tamaño).
            let return_addr = self.pc().wrapping_add(self.instruction_size());
            self.set_reg(LR, return_addr);
        }
        let target = self.reg(PC).wrapping_add(arm_branch_offset(instr) as u32);
        self.set_pc(target);
    }

    /// Ejecuta `BX Rn` (Mini-Hito 2.2e): salta a la dirección de `Rn` y cambia el
    /// estado de ejecución según su bit 0 (1 = THUMB, 0 = ARM) — es lo que activa
    /// por primera vez el modo THUMB. El `PC` se alinea al ancho del nuevo estado.
    fn execute_bx(&mut self, instr: u32) {
        let rn = (instr & 0xF) as usize;
        let target = self.reg(rn);
        let to_thumb = (target & 1) != 0;
        self.cpsr.set_thumb(to_thumb);
        let aligned = if to_thumb { target & !1 } else { target & !3 };
        self.set_pc(aligned);
    }

    /// El CPSR actual (copia; es `Copy`).
    pub fn cpsr(&self) -> Cpsr {
        self.cpsr
    }
    /// Acceso mutable al CPSR para que la ejecución de instrucciones actualice
    /// los flags. No permite cambiar el modo (sus bits solo se tocan vía
    /// [`Cpu::set_mode`], porque cambiar de modo implica intercambiar bancos).
    pub fn cpsr_mut(&mut self) -> &mut Cpsr {
        &mut self.cpsr
    }

    /// El modo de CPU actual, leído de los bits `M[4:0]` del CPSR (única fuente
    /// de verdad: así no puede desincronizarse con un campo aparte).
    pub fn mode(&self) -> CpuMode {
        match CpuMode::from_bits(self.cpsr.mode_bits()) {
            Some(mode) => mode,
            None => {
                // En esta fase, todo cambio de modo pasa por `set_mode`, que
                // recibe un `CpuMode` válido, así que esto no debería ocurrir.
                debug_assert!(false, "bits de modo inválidos en el CPSR");
                CpuMode::System
            }
        }
    }

    /// El `SPSR` del modo actual, o `None` en User/System (que no tienen).
    pub fn spsr(&self) -> Option<u32> {
        let mode = self.mode();
        mode.has_spsr().then(|| self.spsr[mode.bank()])
    }

    /// Escribe el `SPSR` del modo actual. En User/System no hay SPSR, así que la
    /// escritura se descarta silenciosamente (como en el hardware real).
    pub fn set_spsr(&mut self, value: u32) {
        let mode = self.mode();
        if mode.has_spsr() {
            self.spsr[mode.bank()] = value;
        }
    }

    /// Cambia el modo de la CPU **intercambiando los bancos de registros**.
    ///
    /// Este es el corazón de la trampa de diseño que advierte el plan: al cambiar
    /// de modo, `r13`/`r14` (y en FIQ también `r8`–`r12`) pasan a ser otros. El
    /// procedimiento:
    ///
    /// 1. Si el modo no cambia de **banco**, no hay nada que intercambiar (caso
    ///    User↔System, que comparten registros): solo se actualizan los bits.
    /// 2. Si cruza la frontera con **FIQ**, se intercambian además `r8`–`r12`.
    /// 3. Se guarda el `SP`/`LR` visible en el banco viejo y se carga el nuevo.
    /// 4. Se reflejan los bits del nuevo modo en el CPSR.
    ///
    /// (El `SPSR` no se "intercambia": se guarda por banco y se accede por el
    /// modo actual en [`Cpu::spsr`], así que no necesita tratamiento aquí.)
    pub fn set_mode(&mut self, new_mode: CpuMode) {
        let old_mode = self.mode();
        if old_mode == new_mode {
            return;
        }

        // (2) Banking de r8..r12: solo ocurre al cruzar la frontera FIQ/no-FIQ.
        let was_fiq = old_mode == CpuMode::Fiq;
        let is_fiq = new_mode == CpuMode::Fiq;
        if was_fiq != is_fiq {
            if is_fiq {
                // Entramos en FIQ: guardamos los r8..r12 compartidos y cargamos
                // los propios de FIQ.
                self.usr_r8_r12.copy_from_slice(&self.r[8..13]);
                self.r[8..13].copy_from_slice(&self.fiq_r8_r12);
            } else {
                // Salimos de FIQ: guardamos los de FIQ y restauramos los
                // compartidos.
                self.fiq_r8_r12.copy_from_slice(&self.r[8..13]);
                self.r[8..13].copy_from_slice(&self.usr_r8_r12);
            }
        }

        // (3) Banking de SP/LR: solo si cambia de banco (User↔System no lo hace).
        let (old_bank, new_bank) = (old_mode.bank(), new_mode.bank());
        if old_bank != new_bank {
            self.bank_sp[old_bank] = self.r[SP];
            self.bank_lr[old_bank] = self.r[LR];
            self.r[SP] = self.bank_sp[new_bank];
            self.r[LR] = self.bank_lr[new_bank];
        }

        // (4) Reflejar el nuevo modo en el CPSR.
        self.cpsr.set_mode_bits(new_mode.bits());
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

/// Suma `a + b + carry_in` devolviendo `(resultado, carry_out, overflow_con_signo)`.
///
/// La resta se modela como `a + !b + 1` (y `SBC`/`RSC` como `a + !b + carry`), así
/// que esta única función cubre todas las operaciones aritméticas de la ALU. El
/// overflow con signo se detecta cuando ambos operandos comparten signo y el
/// resultado lo cambia; con `b = !b` esa misma fórmula da el overflow de la resta.
fn alu_add(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let carry = carry_in as u32;
    let result = a.wrapping_add(b).wrapping_add(carry);
    let carry_out = (a as u64) + (b as u64) + (carry as u64) > 0xFFFF_FFFF;
    let overflow = (!(a ^ b) & (a ^ result) & 0x8000_0000) != 0;
    (result, carry_out, overflow)
}

/// Adapta la tripleta de [`alu_add`] al formato `(resultado, carry, Some(V))` que
/// espera el `match` de la ALU (las operaciones lógicas usan `None` para no tocar
/// el flag V).
fn with_v(t: (u32, bool, bool)) -> (u32, bool, Option<bool>) {
    (t.0, t.1, Some(t.2))
}

/// `true` si el bit `n` (0 = el menos significativo) de `value` está a 1. `n`
/// debe ser `< 32` (los llamadores lo garantizan).
#[inline]
fn bit(value: u32, n: u32) -> bool {
    ((value >> n) & 1) != 0
}

/// Ciclos internos `m` del multiplicador de Booth del ARM7TDMI (Mini-Hito 2.2h),
/// que **termina antes** cuanto más pequeño es —en magnitud— el multiplicador
/// `Rs`. Es la fuente del «coste en ciclos variable según el operando».
///
/// `m` vale 1, 2, 3 o 4 según cuántos bytes altos de `Rs` sean homogéneos:
/// - `m = 1` si los bits 31-8 son todos iguales (ver `allow_all_ones`),
/// - `m = 2` si lo son los bits 31-16,
/// - `m = 3` si lo son los bits 31-24,
/// - `m = 4` en cualquier otro caso.
///
/// `allow_all_ones` separa la terminación **con signo** de la **sin signo**: con
/// signo (`MUL`/`MLA` y `SMULL`/`SMLAL`) termina pronto tanto con los bits altos
/// «todo ceros» como «todo unos» (el relleno de signo de un negativo pequeño);
/// sin signo (`UMULL`/`UMLAL`) solo con «todo ceros».
fn multiply_internal_cycles(multiplier: u32, allow_all_ones: bool) -> u64 {
    // `high` son los bits altos examinados; homogéneo = todo ceros (o todo unos,
    // que es `high == mask`, si la variante lo permite).
    let homogeneous = |high: u32, mask: u32| high == 0 || (allow_all_ones && high == mask);
    if homogeneous(multiplier & 0xFFFF_FF00, 0xFFFF_FF00) {
        1
    } else if homogeneous(multiplier & 0xFFFF_0000, 0xFFFF_0000) {
        2
    } else if homogeneous(multiplier & 0xFF00_0000, 0xFF00_0000) {
        3
    } else {
        4
    }
}

/// Tipo de desplazamiento del *barrel shifter* (bits 6-5 de una instrucción de
/// procesamiento de datos con operando de registro).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShiftType {
    /// Lógico a la izquierda (`LSL`): rellena con ceros por la derecha.
    Lsl,
    /// Lógico a la derecha (`LSR`): rellena con ceros por la izquierda.
    Lsr,
    /// Aritmético a la derecha (`ASR`): replica el bit de signo por la izquierda.
    Asr,
    /// Rotación a la derecha (`ROR`): los bits que salen por la derecha vuelven a
    /// entrar por la izquierda.
    Ror,
}

impl ShiftType {
    /// Interpreta los 2 bits de tipo de shift (los más bajos de `bits`).
    fn from_bits(bits: u32) -> ShiftType {
        match bits & 0b11 {
            0 => ShiftType::Lsl,
            1 => ShiftType::Lsr,
            2 => ShiftType::Asr,
            3 => ShiftType::Ror,
            _ => unreachable!("bits & 0b11 está en 0..=3"),
        }
    }
}

/// Aplica un desplazamiento por **cantidad inmediata** (`amount` ∈ 0..=31) y
/// devuelve `(resultado, carry_out)`.
///
/// ⚠️ Las **codificaciones especiales de cantidad 0** del ARM7TDMI son la trampa
/// clásica del shifter:
/// - `LSL #0`: el valor pasa intacto y el carry se conserva (es el "sin shift").
/// - `LSR #0` significa en realidad `LSR #32`: todo se desplaza fuera (resultado
///   0) y el carry es el bit 31.
/// - `ASR #0` significa `ASR #32`: el resultado es el bit de signo replicado y el
///   carry, también el bit 31.
/// - `ROR #0` es `RRX`: rota 1 bit a la derecha **a través del carry** (el carry
///   entra por el bit 31 y sale el bit 0).
fn shift_by_immediate(ty: ShiftType, amount: u32, value: u32, carry_in: bool) -> (u32, bool) {
    match ty {
        ShiftType::Lsl => {
            if amount == 0 {
                (value, carry_in)
            } else {
                (value << amount, bit(value, 32 - amount))
            }
        }
        ShiftType::Lsr => {
            if amount == 0 {
                (0, bit(value, 31)) // LSR #0 ≡ LSR #32
            } else {
                (value >> amount, bit(value, amount - 1))
            }
        }
        ShiftType::Asr => {
            if amount == 0 {
                // ASR #0 ≡ ASR #32: el signo se replica a los 32 bits.
                let sign = bit(value, 31);
                (if sign { 0xFFFF_FFFF } else { 0 }, sign)
            } else {
                ((value as i32 >> amount) as u32, bit(value, amount - 1))
            }
        }
        ShiftType::Ror => {
            if amount == 0 {
                // ROR #0 ≡ RRX: rota 1 bit a través del carry.
                (((carry_in as u32) << 31) | (value >> 1), bit(value, 0))
            } else {
                (value.rotate_right(amount), bit(value, amount - 1))
            }
        }
    }
}

/// Aplica un desplazamiento por **cantidad en registro** (`amount` = byte bajo de
/// `Rs`, ∈ 0..=255) y devuelve `(resultado, carry_out)`.
///
/// A diferencia del inmediato, la cantidad 0 **no** tiene codificación especial:
/// deja el valor y el carry intactos. Y modela las cantidades `>= 32`, donde el
/// resultado se satura (todo desplazado fuera, o el signo replicado en `ASR`).
fn shift_by_register(ty: ShiftType, amount: u32, value: u32, carry_in: bool) -> (u32, bool) {
    if amount == 0 {
        return (value, carry_in);
    }
    match ty {
        ShiftType::Lsl => match amount {
            1..=31 => (value << amount, bit(value, 32 - amount)),
            32 => (0, bit(value, 0)),
            _ => (0, false), // > 32: todo fuera, carry 0
        },
        ShiftType::Lsr => match amount {
            1..=31 => (value >> amount, bit(value, amount - 1)),
            32 => (0, bit(value, 31)),
            _ => (0, false),
        },
        ShiftType::Asr => {
            if amount >= 32 {
                let sign = bit(value, 31);
                (if sign { 0xFFFF_FFFF } else { 0 }, sign)
            } else {
                ((value as i32 >> amount) as u32, bit(value, amount - 1))
            }
        }
        ShiftType::Ror => {
            // ROR por más de 32 equivale a ROR por (amount mód 32). Si el módulo
            // es 0 (amount múltiplo de 32), el valor queda igual y el carry es el
            // bit 31.
            let r = amount & 0x1F;
            if r == 0 {
                (value, bit(value, 31))
            } else {
                (value.rotate_right(r), bit(value, r - 1))
            }
        }
    }
}

/// Decodifica el desplazamiento de un salto ARM `B`/`BL`: los 24 bits bajos son
/// un offset con signo en palabras, así que se extiende el signo y se multiplica
/// por 4. (Lo reutilizará la ejecución real de saltos en el Mini-Hito 2.2e.)
fn arm_branch_offset(instr: u32) -> i32 {
    (((instr & 0x00FF_FFFF) << 8) as i32) >> 6
}

/// `true` si `instr` es un salto ARM (`B`/`BL`) cuyo destino es su propia
/// dirección (`b .`): un bucle infinito de una sola instrucción. `pc` es la
/// dirección cruda de la instrucción; el destino se calcula sobre el `PC`
/// adelantado por el pipeline (+8 en ARM), igual que hará la ejecución real.
fn is_branch_to_self(kind: ArmInstruction, instr: u32, pc: u32) -> bool {
    matches!(kind, ArmInstruction::Branch { .. })
        && pc
            .wrapping_add(PC_AHEAD_ARM)
            .wrapping_add(arm_branch_offset(instr) as u32)
            == pc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn estado_de_reset_es_supervisor_arm_irq_fiq_off() {
        let cpu = Cpu::new();
        assert_eq!(cpu.mode(), CpuMode::Supervisor);
        assert!(!cpu.cpsr().thumb(), "arranca en estado ARM, no THUMB");
        assert!(cpu.cpsr().irq_disabled());
        assert!(cpu.cpsr().fiq_disabled());
        // r0..=r14 a cero. r15 también es 0 en crudo (`pc()`); `reg(PC)` no se
        // comprueba aquí porque le añade el desfase de pipeline (+8 en ARM).
        for i in 0..PC {
            assert_eq!(cpu.reg(i), 0);
        }
        assert_eq!(cpu.pc(), 0);
    }

    #[test]
    fn lee_y_escribe_registros() {
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0xDEAD_BEEF);
        cpu.set_pc(0x0800_0000);
        assert_eq!(cpu.reg(0), 0xDEAD_BEEF);
        // `pc()` da el valor crudo; `reg(PC)` lo ve adelantado por el pipeline
        // (+8 en ARM, el estado de reset). Ver `el_pipeline_adelanta_r15_en_arm`.
        assert_eq!(cpu.pc(), 0x0800_0000);
        assert_eq!(cpu.reg(PC), 0x0800_0008);
    }

    #[test]
    fn el_pipeline_adelanta_r15_en_arm() {
        // En estado ARM (el de reset), leer r15 como operando ve PC + 8.
        let mut cpu = Cpu::new();
        assert!(!cpu.cpsr().thumb());
        cpu.set_pc(0x0800_0000);
        assert_eq!(cpu.pipeline_offset(), PC_AHEAD_ARM);
        assert_eq!(cpu.pc(), 0x0800_0000, "pc() no lleva desfase");
        assert_eq!(cpu.reg(PC), 0x0800_0008, "reg(PC) va dos instrucciones por delante");
    }

    #[test]
    fn el_pipeline_adelanta_r15_en_thumb() {
        // En THUMB las instrucciones son de 2 bytes: el desfase es +4.
        let mut cpu = Cpu::new();
        cpu.cpsr_mut().set_thumb(true);
        cpu.set_pc(0x0800_0000);
        assert_eq!(cpu.pipeline_offset(), PC_AHEAD_THUMB);
        assert_eq!(cpu.pc(), 0x0800_0000);
        assert_eq!(cpu.reg(PC), 0x0800_0004);
    }

    #[test]
    fn escribir_r15_guarda_el_valor_crudo() {
        // Escribir r15 (p. ej. un salto) fija la dirección destino sin desfase;
        // el desfase solo aparece al LEERlo de vuelta como operando.
        let mut cpu = Cpu::new();
        cpu.set_reg(PC, 0x0800_1000);
        assert_eq!(cpu.pc(), 0x0800_1000);
        assert_eq!(cpu.reg(PC), 0x0800_1008); // +8 ARM al releer
    }

    #[test]
    fn los_demas_registros_no_llevan_desfase() {
        // El desfase es exclusivo de r15; r0..=r14 se leen tal cual.
        let mut cpu = Cpu::new();
        for i in 0..PC {
            cpu.set_reg(i, 0x1000 + i as u32);
        }
        for i in 0..PC {
            assert_eq!(cpu.reg(i), 0x1000 + i as u32);
        }
    }

    #[test]
    fn los_flags_del_cpsr_se_ponen_y_se_leen() {
        let mut cpu = Cpu::new();
        let cpsr = cpu.cpsr_mut();
        cpsr.set_n(true);
        cpsr.set_z(true);
        cpsr.set_c(false);
        cpsr.set_v(true);
        let cpsr = cpu.cpsr();
        assert!(cpsr.n() && cpsr.z() && cpsr.v());
        assert!(!cpsr.c());
    }

    #[test]
    fn set_mode_banca_sp_y_lr_por_modo() {
        let mut cpu = Cpu::new(); // arranca en Supervisor
        cpu.set_reg(SP, 0x0300_7F00); // SP del Supervisor

        cpu.set_mode(CpuMode::Irq);
        // El IRQ tiene su propio SP (aún 0); el del Supervisor quedó guardado.
        assert_eq!(cpu.sp(), 0);
        cpu.set_reg(SP, 0x0300_7FA0); // SP del IRQ

        cpu.set_mode(CpuMode::Supervisor);
        // Al volver, recuperamos el SP del Supervisor intacto.
        assert_eq!(cpu.sp(), 0x0300_7F00);

        cpu.set_mode(CpuMode::Irq);
        assert_eq!(cpu.sp(), 0x0300_7FA0); // y el del IRQ también
    }

    #[test]
    fn user_y_system_comparten_sp_y_lr() {
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::System);
        cpu.set_reg(SP, 0x0300_7E00);
        cpu.set_mode(CpuMode::User);
        // Comparten banco: el SP NO se intercambia.
        assert_eq!(cpu.sp(), 0x0300_7E00);
    }

    #[test]
    fn fiq_banca_tambien_r8_a_r12() {
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::System);
        cpu.set_reg(8, 0x1111_1111); // r8 "compartido" por los modos no-FIQ

        cpu.set_mode(CpuMode::Fiq);
        assert_eq!(cpu.reg(8), 0, "FIQ ve su propio r8");
        cpu.set_reg(8, 0x2222_2222);

        cpu.set_mode(CpuMode::System);
        assert_eq!(cpu.reg(8), 0x1111_1111, "al salir de FIQ se restaura r8");

        cpu.set_mode(CpuMode::Fiq);
        assert_eq!(cpu.reg(8), 0x2222_2222, "FIQ conserva su r8 bancado");
    }

    #[test]
    fn solo_los_modos_de_excepcion_tienen_spsr() {
        let mut cpu = Cpu::new(); // Supervisor: sí tiene
        cpu.set_spsr(0xCAFE_0000);
        assert_eq!(cpu.spsr(), Some(0xCAFE_0000));

        cpu.set_mode(CpuMode::User); // User: no tiene
        assert_eq!(cpu.spsr(), None);
        cpu.set_spsr(0x1234); // se descarta sin panicar
        assert_eq!(cpu.spsr(), None);

        // El SPSR del Supervisor sigue intacto al volver.
        cpu.set_mode(CpuMode::Supervisor);
        assert_eq!(cpu.spsr(), Some(0xCAFE_0000));
    }

    #[test]
    fn el_modo_queda_reflejado_en_los_bits_del_cpsr() {
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::Irq);
        assert_eq!(cpu.cpsr().mode_bits(), CpuMode::Irq.bits());
        assert_eq!(cpu.cpsr().mode_bits(), 0x12);
    }

    #[test]
    fn fetch_lee_la_instruccion_de_32_bits_en_little_endian() {
        // 0xEA00002E es el ejemplo del plan; en little-endian son los bytes
        // [0x2E, 0x00, 0x00, 0xEA] al inicio de la ROM.
        let mut rom = vec![0u8; 8];
        rom[..4].copy_from_slice(&[0x2E, 0x00, 0x00, 0xEA]);
        let bus = Bus::new(rom);

        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);

        assert_eq!(cpu.fetch(&bus), 0xEA00_002E);
    }

    #[test]
    fn decode_arm_usa_el_cpsr_para_la_condicion() {
        use crate::arm::Condition;

        let mut cpu = Cpu::new();
        // "BEQ": salta solo si Z = 1. Con Z = 0 (reset) se descarta como NOP.
        assert_eq!(
            cpu.decode_arm(0x0A00_002E),
            Decoded::ConditionFailed(Condition::Eq)
        );
        // Activando Z, la misma instrucción ya se identifica como salto.
        cpu.cpsr_mut().set_z(true);
        assert_eq!(
            cpu.decode_arm(0x0A00_002E),
            Decoded::Execute(ArmInstruction::Branch { link: false })
        );
    }

    #[test]
    fn decode_thumb_no_depende_del_cpsr() {
        // El decode THUMB clasifica directo, sin importar los flags del CPSR.
        let mut cpu = Cpu::new();
        cpu.cpsr_mut().set_z(true);
        cpu.cpsr_mut().set_n(true);
        // 0x2005 = «MOV r0, #5» en THUMB (formato 3).
        assert_eq!(cpu.decode_thumb(0x2005), ThumbInstruction::MoveCompareAddSubImm);
    }

    #[test]
    fn mov_inmediato_escribe_el_registro() {
        // La "Prueba" del plan: MOV R0, #5 (0xE3A00005) deja R0 == 5.
        let mut cpu = Cpu::new();
        cpu.execute_data_processing(0xE3A0_0005);
        assert_eq!(cpu.reg(0), 5);
    }

    #[test]
    fn add_inmediato_suma_sobre_el_registro() {
        // ADD R1, R1, #1 (0xE2811001) con R1 = 10 → 11.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 10);
        cpu.execute_data_processing(0xE281_1001);
        assert_eq!(cpu.reg(1), 11);
    }

    #[test]
    fn operando_inmediato_rotado() {
        // MOV R0, #0xFF rotado a la derecha 8 bits (0xE3A004FF) → 0xFF000000.
        let mut cpu = Cpu::new();
        cpu.execute_data_processing(0xE3A0_04FF);
        assert_eq!(cpu.reg(0), 0xFF00_0000);
    }

    #[test]
    fn movs_y_mvns_actualizan_n_y_z() {
        let mut cpu = Cpu::new();
        // MOVS R0, #0 (0xE3B00000) → R0 = 0, Z = 1, N = 0.
        cpu.execute_data_processing(0xE3B0_0000);
        assert_eq!(cpu.reg(0), 0);
        assert!(cpu.cpsr().z());
        assert!(!cpu.cpsr().n());
        // MVNS R0, #0 (0xE3F00000) → R0 = 0xFFFFFFFF, N = 1, Z = 0.
        cpu.execute_data_processing(0xE3F0_0000);
        assert_eq!(cpu.reg(0), 0xFFFF_FFFF);
        assert!(cpu.cpsr().n());
        assert!(!cpu.cpsr().z());
    }

    #[test]
    fn cmp_actualiza_flags_sin_escribir_rd() {
        // CMP R0, #5 (0xE3500005) con R0 = 5 → Z = 1, C = 1, y R0 no cambia.
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 5);
        cpu.execute_data_processing(0xE350_0005);
        assert!(cpu.cpsr().z(), "5 - 5 == 0 → Z");
        assert!(cpu.cpsr().c(), "5 >= 5 → sin borrow → C");
        assert_eq!(cpu.reg(0), 5, "CMP no escribe Rd");
    }

    #[test]
    fn subs_marca_signo_y_borrow() {
        // SUBS R0, R0, #1 (0xE2500001) con R0 = 0 → 0xFFFFFFFF, N=1, Z=0, C=0.
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0);
        cpu.execute_data_processing(0xE250_0001);
        assert_eq!(cpu.reg(0), 0xFFFF_FFFF);
        assert!(cpu.cpsr().n());
        assert!(!cpu.cpsr().z());
        assert!(!cpu.cpsr().c(), "0 - 1 genera borrow → C = 0");
    }

    #[test]
    fn adc_usa_el_carry_de_entrada() {
        // ADC R0, R0, #0 (0xE2A00000) con R0 = 0 y C = 1 → R0 = 1.
        let mut cpu = Cpu::new();
        cpu.cpsr_mut().set_c(true);
        cpu.execute_data_processing(0xE2A0_0000);
        assert_eq!(cpu.reg(0), 1);
    }

    #[test]
    fn el_paso_trata_la_condicion_fallida_como_nop() {
        // BEQ con Z = 0 (reset): la condición falla → NOP de un ciclo que solo
        // avanza el PC, sin detener el bucle.
        let mut rom = vec![0u8; 8];
        rom[0..4].copy_from_slice(&0x0A00_0000u32.to_le_bytes()); // 0000 = EQ
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);

        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.pc(), crate::bus::ROM_START + 4, "el NOP avanza una instrucción");
    }

    #[test]
    fn el_bucle_ejecuta_hasta_una_no_implementada() {
        use crate::arm::ArmInstruction;
        // MOV r0,#5 ; ADD r0,r0,#1 ; LDR r2,[r0] (carga: aún sin implementar).
        let programa = [0xE3A0_0005u32, 0xE280_0001, 0xE590_2000];
        let mut rom = vec![0u8; programa.len() * 4];
        for (i, w) in programa.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);

        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.reg(0), 5); // MOV r0, #5
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.reg(0), 6); // ADD r0, r0, #1

        // La tercera (una carga) aún no está implementada → la CPU se detiene.
        let pc_culpable = cpu.pc();
        match cpu.step(&bus) {
            StepResult::Halted(Halt::Unimplemented { pc, instr, kind }) => {
                assert_eq!(pc, crate::bus::ROM_START + 8);
                assert_eq!(instr, 0xE590_2000);
                assert_eq!(kind, ArmInstruction::SingleDataTransfer);
            }
            otro => panic!("esperaba Halted, fue {otro:?}"),
        }
        // El PC NO avanzó: sigue apuntando a la instrucción no implementada.
        assert_eq!(cpu.pc(), pc_culpable);
    }

    #[test]
    fn data_processing_con_registro_se_ejecuta() {
        // MOV r0, r1 (forma con registro, bit 25 = 0): 0xE1A00001. Desde el
        // Mini-Hito 2.2f ya se ejecuta (barrel shifter), en vez de detener el bucle.
        let mut rom = vec![0u8; 8];
        rom[0..4].copy_from_slice(&0xE1A0_0001u32.to_le_bytes());
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);
        cpu.set_reg(1, 0x1234_5678);
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.reg(0), 0x1234_5678);
    }

    #[test]
    fn run_para_al_alcanzar_el_tope_de_pasos() {
        // ROM de ceros: 0x00000000 es cond EQ (falla en reset) → NOP infinito.
        // El tope debe cortar el bucle en seco.
        let bus = Bus::new(vec![0u8; 64]);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);
        let report = cpu.run(&bus, 10);
        assert_eq!(report.steps, 10);
        assert_eq!(report.stop, RunStop::StepLimit);
    }

    #[test]
    fn detecta_el_bucle_infinito_b_a_si_mismo() {
        // 0xEAFFFFFE = «b .» (salto a su propia dirección): la señal de "fin"
        // de las ROMs de test. Se reconoce sin necesidad de ejecutar el salto.
        let mut rom = vec![0u8; 8];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);
        match cpu.step(&bus) {
            StepResult::Halted(Halt::InfiniteLoop { pc, instr }) => {
                assert_eq!(pc, crate::bus::ROM_START);
                assert_eq!(instr, 0xEAFF_FFFE);
            }
            otro => panic!("esperaba InfiniteLoop, fue {otro:?}"),
        }
    }

    #[test]
    fn b_no_a_si_mismo_se_ejecuta_y_salta() {
        // 0xEA00002E (el salto de arranque de las ROMs reales) ya NO es "no
        // implementado": se ejecuta y salta a pc+8 + (0x2E×4) = pc + 0xC0.
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xEA00_002E);
        let mut cpu = Cpu::new();
        cpu.set_pc(base);
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.pc(), base + 0xC0);
    }

    #[test]
    fn b_salta_hacia_delante_y_se_salta_instrucciones() {
        // [0] B → base+8 (se salta [1]); [1] MOV r0,#0xFF; [2] MOV r1,#1.
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xEA00_0000); // B con offset 0 → destino = pc+8 = [2]
        bus.write_u32(base + 4, 0xE3A0_00FF); // MOV r0, #0xFF (debe saltarse)
        bus.write_u32(base + 8, 0xE3A0_1001); // MOV r1, #1
        let mut cpu = Cpu::new();
        cpu.set_pc(base);

        assert_eq!(cpu.step(&bus), StepResult::Stepped); // B
        assert_eq!(cpu.pc(), base + 8);
        assert_eq!(cpu.step(&bus), StepResult::Stepped); // MOV r1, #1
        assert_eq!(cpu.reg(1), 1);
        assert_eq!(cpu.reg(0), 0, "la instrucción saltada no se ejecutó");
    }

    #[test]
    fn bl_y_bx_lr_vuelven_del_subprograma() {
        // [0] BL [3] ; [1] MOV r0,#1 (al volver) ; [2] relleno ;
        // [3] MOV r2,#2 ; [4] BX lr
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xEB00_0001); // BL → base+12 ([3]); LR = base+4
        bus.write_u32(base + 4, 0xE3A0_0001); // MOV r0, #1 (tras volver)
        bus.write_u32(base + 8, 0xE3A0_00AA); // relleno (no se ejecuta)
        bus.write_u32(base + 12, 0xE3A0_2002); // [3] MOV r2, #2
        bus.write_u32(base + 16, 0xE12F_FF1E); // [4] BX lr
        let mut cpu = Cpu::new();
        cpu.set_pc(base);

        cpu.step(&bus); // BL
        assert_eq!(cpu.pc(), base + 12, "BL salta a la subrutina");
        assert_eq!(cpu.reg(LR), base + 4, "BL guarda el retorno en LR");

        cpu.step(&bus); // MOV r2, #2
        assert_eq!(cpu.reg(2), 2);

        cpu.step(&bus); // BX lr
        assert_eq!(cpu.pc(), base + 4, "BX lr vuelve a la instrucción siguiente al BL");
        assert!(!cpu.cpsr().thumb(), "LR con bit0=0 → sigue en ARM");

        cpu.step(&bus); // MOV r0, #1
        assert_eq!(cpu.reg(0), 1);
    }

    #[test]
    fn bx_a_thumb_cambia_de_estado_y_se_detiene() {
        // BX a una dirección impar → estado THUMB. Como la ejecución THUMB aún no
        // existe (2.2m/2.3a), el siguiente paso se detiene limpiamente.
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xE12F_FF10); // BX r0
        let mut cpu = Cpu::new();
        cpu.set_pc(base);
        cpu.set_reg(0, base + 0x101); // destino impar (bit0=1) → THUMB

        cpu.step(&bus); // BX → THUMB, PC = (base+0x101) & !1 = base+0x100
        assert!(cpu.cpsr().thumb());
        assert_eq!(cpu.pc(), base + 0x100);
        assert!(matches!(
            cpu.step(&bus),
            StepResult::Halted(Halt::ThumbNotImplemented { .. })
        ));
    }

    #[test]
    fn el_contador_de_ciclos_suma_cada_fetch() {
        // Tres MOV inmediatos en IWRAM (bus de 32 bits, 0 waits → 1 ciclo/fetch).
        let mut bus = Bus::new(vec![0u8; 4]);
        for i in 0..3u32 {
            bus.write_u32(crate::bus::IWRAM_START + i * 4, 0xE3A0_0000 | i);
        }
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);
        assert_eq!(cpu.cycles(), 0);

        for _ in 0..3 {
            assert_eq!(cpu.step(&bus), StepResult::Stepped);
        }
        // 3 instrucciones × 1 ciclo (IWRAM no distingue N de S: ambos cuestan 1).
        assert_eq!(cpu.cycles(), 3);
    }

    #[test]
    fn los_ciclos_dependen_de_la_region() {
        // El mismo MOV cuesta más desde EWRAM (bus de 16 bits + 2 waits → un fetch
        // de 32 bits son 2 sub-accesos = 6 ciclos) que desde IWRAM (1 ciclo).
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(crate::bus::IWRAM_START, 0xE3A0_0000);
        bus.write_u32(crate::bus::EWRAM_START, 0xE3A0_0000);

        let mut en_iwram = Cpu::new();
        en_iwram.set_pc(crate::bus::IWRAM_START);
        en_iwram.step(&bus);
        assert_eq!(en_iwram.cycles(), 1);

        let mut en_ewram = Cpu::new();
        en_ewram.set_pc(crate::bus::EWRAM_START);
        en_ewram.step(&bus);
        assert_eq!(en_ewram.cycles(), 6);
    }

    #[test]
    fn run_reporta_los_ciclos_consumidos() {
        // Dos MOV en IWRAM y una tercera no implementada: `run` cuenta solo los
        // ciclos de los dos pasos ejecutados (2 × 1 = 2).
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(crate::bus::IWRAM_START, 0xE3A0_0000); // MOV r0, #0
        bus.write_u32(crate::bus::IWRAM_START + 4, 0xE3A0_1001); // MOV r1, #1
        bus.write_u32(crate::bus::IWRAM_START + 8, 0xE590_2000); // LDR r2, [r0]: no impl.
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);

        let report = cpu.run(&bus, 100);
        assert_eq!(report.steps, 2);
        assert_eq!(report.cycles, 2);
        assert!(matches!(
            report.stop,
            RunStop::Halted(Halt::Unimplemented { .. })
        ));
    }

    // ===== Barrel shifter (Mini-Hito 2.2f) =================================

    #[test]
    fn shifter_lsl_inmediato() {
        // LSL #4: carry = bit (32-4) = bit 28.
        assert_eq!(shift_by_immediate(ShiftType::Lsl, 4, 0x0000_000F, false), (0xF0, false));
        // LSL #0: valor y carry intactos (es el "sin shift").
        assert_eq!(shift_by_immediate(ShiftType::Lsl, 0, 0x8000_0000, true), (0x8000_0000, true));
        // LSL #1 de 0x8000_0000: sale el bit 31 → carry = 1, resultado 0.
        assert_eq!(shift_by_immediate(ShiftType::Lsl, 1, 0x8000_0000, false), (0, true));
    }

    #[test]
    fn shifter_lsr_inmediato_y_el_cero_es_32() {
        // LSR #4 de 0xFF → 0x0F, carry = bit 3 = 1.
        assert_eq!(shift_by_immediate(ShiftType::Lsr, 4, 0x0000_00FF, false), (0x0F, true));
        // LSR #0 ≡ LSR #32: todo fuera (0), carry = bit 31.
        assert_eq!(shift_by_immediate(ShiftType::Lsr, 0, 0x8000_0000, false), (0, true));
        assert_eq!(shift_by_immediate(ShiftType::Lsr, 0, 0x7FFF_FFFF, true), (0, false));
    }

    #[test]
    fn shifter_asr_inmediato_replica_signo_y_el_cero_es_32() {
        // ASR #4 de 0x8000_0000 → 0xF800_0000 (signo replicado), carry = bit 3 = 0.
        assert_eq!(shift_by_immediate(ShiftType::Asr, 4, 0x8000_0000, false), (0xF800_0000, false));
        // ASR #0 ≡ ASR #32: el bit de signo se replica a los 32 bits.
        assert_eq!(shift_by_immediate(ShiftType::Asr, 0, 0x8000_0000, false), (0xFFFF_FFFF, true));
        assert_eq!(shift_by_immediate(ShiftType::Asr, 0, 0x4000_0000, false), (0, false));
    }

    #[test]
    fn shifter_ror_inmediato_y_rrx() {
        // ROR #4 de 0x0000_000F → 0xF000_0000, carry = bit 3 = 1.
        assert_eq!(shift_by_immediate(ShiftType::Ror, 4, 0x0000_000F, false), (0xF000_0000, true));
        // ROR #0 ≡ RRX: el carry entra por el bit 31 y sale el bit 0.
        assert_eq!(shift_by_immediate(ShiftType::Ror, 0, 0x0000_0001, false), (0, true));
        assert_eq!(shift_by_immediate(ShiftType::Ror, 0, 0x0000_0000, true), (0x8000_0000, false));
    }

    #[test]
    fn shifter_por_registro_cantidad_cero_no_cambia_nada() {
        // A diferencia del inmediato, la cantidad 0 por registro no tiene
        // codificación especial: valor y carry intactos, sea cual sea el tipo.
        assert_eq!(shift_by_register(ShiftType::Lsl, 0, 0x1234_5678, true), (0x1234_5678, true));
        assert_eq!(shift_by_register(ShiftType::Ror, 0, 0x1234_5678, false), (0x1234_5678, false));
    }

    #[test]
    fn shifter_por_registro_casos_limite_de_32_y_mas() {
        // LSL 32: 0, carry = bit 0. LSL > 32: 0, carry 0.
        assert_eq!(shift_by_register(ShiftType::Lsl, 32, 0x0000_0001, false), (0, true));
        assert_eq!(shift_by_register(ShiftType::Lsl, 33, 0xFFFF_FFFF, true), (0, false));
        // LSR 32: 0, carry = bit 31.
        assert_eq!(shift_by_register(ShiftType::Lsr, 32, 0x8000_0000, false), (0, true));
        // ASR >= 32: signo replicado, carry = bit 31.
        assert_eq!(shift_by_register(ShiftType::Asr, 50, 0x8000_0000, false), (0xFFFF_FFFF, true));
        assert_eq!(shift_by_register(ShiftType::Asr, 50, 0x0000_0001, false), (0, false));
        // ROR 32: valor intacto, carry = bit 31. ROR 36 ≡ ROR 4.
        assert_eq!(shift_by_register(ShiftType::Ror, 32, 0x8000_0000, false), (0x8000_0000, true));
        assert_eq!(shift_by_register(ShiftType::Ror, 36, 0x0000_000F, false), (0xF000_0000, true));
    }

    // ===== Procesamiento de datos con operando de registro (2.2f) ==========

    #[test]
    fn mov_con_registro_desplazado_por_inmediato() {
        // MOV r0, r1, LSL #4 (0xE1A00201): r1 = 0x12 → r0 = 0x120.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 0x12);
        cpu.execute_data_processing(0xE1A0_0201);
        assert_eq!(cpu.reg(0), 0x120);
    }

    #[test]
    fn movs_lsr_actualiza_el_carry_desde_el_shifter() {
        // MOVS r0, r1, LSR #1 (0xE1B000A1): r1 = 1 → r0 = 0, el bit 0 que sale va
        // al carry, y Z = 1.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 0x0000_0001);
        cpu.execute_data_processing(0xE1B0_00A1);
        assert_eq!(cpu.reg(0), 0);
        assert!(cpu.cpsr().c(), "el bit que sale por LSR #1 va al carry");
        assert!(cpu.cpsr().z());
    }

    #[test]
    fn add_con_operando_de_registro() {
        // ADD r0, r1, r2 (0xE0810002): 10 + 20 = 30.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 10);
        cpu.set_reg(2, 20);
        cpu.execute_data_processing(0xE081_0002);
        assert_eq!(cpu.reg(0), 30);
    }

    #[test]
    fn shift_por_registro_cuesta_un_i_cycle_extra() {
        // En IWRAM (1 ciclo/fetch): la forma por inmediato cuesta 1; la forma por
        // registro, 1 (fetch) + 1 (I-cycle) = 2.
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(crate::bus::IWRAM_START, 0xE1A0_0081); // MOV r0, r1, LSL #1
        let mut por_inmediato = Cpu::new();
        por_inmediato.set_pc(crate::bus::IWRAM_START);
        por_inmediato.step(&bus);
        assert_eq!(por_inmediato.cycles(), 1);

        bus.write_u32(crate::bus::IWRAM_START, 0xE1A0_0211); // MOV r0, r1, LSL r2
        let mut por_registro = Cpu::new();
        por_registro.set_pc(crate::bus::IWRAM_START);
        por_registro.step(&bus);
        assert_eq!(por_registro.cycles(), 2, "el shift por registro añade un I-cycle");
    }

    #[test]
    fn r15_como_operando_va_mas_lejos_con_shift_por_registro() {
        // En ARM, r15 leído como operando es PC+8; pero si la cantidad de shift
        // está en un registro, es PC+12 (el I-cycle adelanta un fetch más).
        let mut cpu = Cpu::new();
        cpu.set_pc(0x0800_0000);
        assert_eq!(cpu.reg_operand(PC, false), 0x0800_0008, "PC+8 normal");
        assert_eq!(cpu.reg_operand(PC, true), 0x0800_000C, "PC+12 con shift por registro");
        // Los registros normales no se ven afectados.
        cpu.set_reg(0, 0xAA);
        assert_eq!(cpu.reg_operand(0, true), 0xAA);
    }

    #[test]
    fn mov_a_pc_es_un_salto_que_alinea_a_palabra() {
        // MOV pc, r0 (0xE1A0F000): el PC pasa a r0 alineado a palabra (ARM) y la
        // ejecución lo reporta como salto (Branched).
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0x0800_1236); // bits bajos sucios a propósito
        let efecto = cpu.execute_data_processing(0xE1A0_F000);
        assert_eq!(cpu.pc(), 0x0800_1234, "destino alineado a palabra en ARM");
        assert!(matches!(efecto, Executed::Branched { .. }));
    }

    #[test]
    fn movs_a_pc_restaura_el_cpsr_desde_el_spsr() {
        // Retorno de excepción: en IRQ con un SPSR que apunta a User (ARM, Z=1),
        // MOVS pc, lr (0xE1B0F00E) vuelve a User, restaura los flags y salta a LR.
        let mut cpu = Cpu::new(); // arranca en Supervisor
        cpu.set_mode(CpuMode::Irq);
        let spsr = (CpuMode::User.bits() as u32) | (1 << 30); // modo User + Z, T=0
        cpu.set_spsr(spsr);
        cpu.set_reg(LR, 0x0800_2000);

        cpu.execute_data_processing(0xE1B0_F00E);
        assert_eq!(cpu.pc(), 0x0800_2000, "salta a LR");
        assert_eq!(cpu.mode(), CpuMode::User, "vuelve al modo guardado en el SPSR");
        assert!(cpu.cpsr().z(), "restaura los flags del SPSR");
        assert!(!cpu.cpsr().thumb(), "el SPSR estaba en estado ARM");
    }

    #[test]
    fn salto_via_pc_vacia_el_pipeline_en_el_bucle() {
        // ADD pc, pc, r0 con shift cero: salto calculado. Verifica que el step lo
        // trata como salto (no avanza secuencialmente) y deja el PC en el destino.
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        // MOV pc, r0 (forma de registro): salta a r0.
        bus.write_u32(base, 0xE1A0_F000);
        let mut cpu = Cpu::new();
        cpu.set_pc(base);
        cpu.set_reg(0, base + 0x40);
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.pc(), base + 0x40, "el PC quedó en el destino del salto");
    }

    // ===== Transferencia de PSR: MRS / MSR (Mini-Hito 2.2g) ================

    #[test]
    fn mrs_lee_el_cpsr_a_un_registro() {
        // MRS r0, CPSR (0xE10F0000): r0 recibe el CPSR completo, flags incluidos.
        let mut cpu = Cpu::new(); // Supervisor + I + F = 0x0000_00D3
        cpu.cpsr_mut().set_n(true); // → 0x8000_00D3
        cpu.execute_psr_transfer(0xE10F_0000);
        assert_eq!(cpu.reg(0), 0x8000_00D3);
    }

    #[test]
    fn mrs_lee_el_spsr_del_modo_actual() {
        // En IRQ (que sí tiene SPSR), MRS r1, SPSR (0xE14F1000) lo copia a r1.
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::Irq);
        cpu.set_spsr(0x8000_0010); // N=1, modo User guardado
        cpu.execute_psr_transfer(0xE14F_1000);
        assert_eq!(cpu.reg(1), 0x8000_0010);
    }

    #[test]
    fn msr_inmediato_cambia_los_flags_y_afecta_a_la_condicion() {
        // MSR CPSR_f, #0xF0000000 (0xE328F20F): pone N,Z,C,V. Es la "Prueba" del
        // plan: cambiar flags vía MSR afecta a las condiciones.
        let mut cpu = Cpu::new();
        cpu.execute_psr_transfer(0xE328_F20F);
        assert!(cpu.cpsr().n() && cpu.cpsr().z() && cpu.cpsr().c() && cpu.cpsr().v());
        // Y una condición que dependa de esos flags ahora se cumple.
        assert!(crate::arm::Condition::Eq.passes(cpu.cpsr()), "Z=1 → EQ pasa");
    }

    #[test]
    fn msr_campo_de_flags_no_toca_el_byte_de_control() {
        // MSR CPSR_f, r0 (0xE128F000) solo escribe el byte de flags: modo/I/F intactos.
        let mut cpu = Cpu::new(); // Supervisor + I + F
        let modo_antes = cpu.cpsr().mode_bits();
        cpu.set_reg(0, 0xF000_0000);
        cpu.execute_psr_transfer(0xE128_F000);
        assert!(cpu.cpsr().n());
        assert_eq!(cpu.cpsr().mode_bits(), modo_antes, "el modo no cambia con el campo f");
        assert!(cpu.cpsr().irq_disabled(), "el bit I sigue como estaba");
    }

    #[test]
    fn msr_campo_de_control_cambia_de_modo_e_intercambia_bancos() {
        // MSR CPSR_c, r0 (0xE121F000) con r0 = modo System: cambia de modo y,
        // como pasa por set_mode, intercambia el banco de SP correctamente.
        let mut cpu = Cpu::new(); // Supervisor
        cpu.set_reg(SP, 0x1111_1111); // SP del Supervisor
        cpu.set_mode(CpuMode::System);
        cpu.set_reg(SP, 0x2222_2222); // SP de System/User
        cpu.set_mode(CpuMode::Supervisor); // volvemos; vemos el SP del SVC

        cpu.set_reg(0, CpuMode::System.bits() as u32); // 0x1F
        cpu.execute_psr_transfer(0xE121_F000);
        assert_eq!(cpu.mode(), CpuMode::System, "el byte de control cambió el modo");
        assert_eq!(cpu.sp(), 0x2222_2222, "set_mode intercambió el banco de SP");
    }

    #[test]
    fn msr_en_modo_user_solo_cambia_los_flags() {
        // En User, MSR CPSR_fc, r0 intenta tocar flags Y control; solo los flags
        // deben cambiar (los de control son de solo lectura en User).
        let mut cpu = Cpu::new(); // Supervisor, I=1
        cpu.set_mode(CpuMode::User); // sigue con I=1
        assert!(cpu.cpsr().irq_disabled());
        // r0 pide N,Z,C,V=1 y, en el byte de control, modo Supervisor con I=0.
        cpu.set_reg(0, 0xF000_0013);
        cpu.execute_psr_transfer(0xE129_F000); // MSR CPSR_fc, r0
        assert!(cpu.cpsr().n(), "los flags SÍ cambian en User");
        assert_eq!(cpu.mode(), CpuMode::User, "el modo NO cambia en User");
        assert!(cpu.cpsr().irq_disabled(), "el bit I (control) NO cambia en User");
    }

    #[test]
    fn msr_escribe_el_spsr_del_modo_actual() {
        // En IRQ, MSR SPSR_f, r0 (0xE168F000) escribe solo el byte de flags del SPSR.
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::Irq);
        cpu.set_spsr(0x0000_0010); // modo User guardado, sin flags
        cpu.set_reg(0, 0xF000_0000);
        cpu.execute_psr_transfer(0xE168_F000);
        assert_eq!(cpu.spsr(), Some(0xF000_0010), "solo cambió el byte de flags del SPSR");
    }

    #[test]
    fn msr_a_spsr_en_user_se_descarta_sin_panicar() {
        // User no tiene SPSR: MSR SPSR_*, r0 no debe hacer nada ni panicar.
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::User);
        cpu.set_reg(0, 0xFFFF_FFFF);
        cpu.execute_psr_transfer(0xE168_F000);
        assert_eq!(cpu.spsr(), None);
    }

    #[test]
    fn msr_no_escribe_los_bits_reservados() {
        // MSR CPSR_fsxc, r0 (0xE12FF000) con casi todos los bits a 1: los bits
        // reservados (27-8) deben quedar a 0; solo NZCV y el byte de control existen.
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0xFFFF_FF13); // flags + reservados + control = Supervisor
        cpu.execute_psr_transfer(0xE12F_F000);
        assert_eq!(cpu.cpsr().bits() & !PSR_VALID, 0, "los bits reservados siguen a 0");
        assert!(cpu.cpsr().n(), "pero los flags sí se escriben");
        assert_eq!(cpu.mode(), CpuMode::Supervisor, "y el modo pedido (0x13) se respeta");
    }

    #[test]
    fn psr_transfer_se_ejecuta_en_el_bucle() {
        // Por el step completo: MSR CPSR_f, #0x40000000 (0xE328F101) pone Z; el
        // decode lo identifica como PsrTransfer y lo ejecuta (antes se detenía).
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xE328_F101);
        let mut cpu = Cpu::new();
        cpu.set_pc(base);
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert!(cpu.cpsr().z(), "MSR puso Z");
        assert_eq!(cpu.pc(), base + 4, "MRS/MSR avanzan el PC como una instrucción normal");
    }

    // ===== Multiplicación: MUL / MLA / largas (Mini-Hito 2.2h) =============

    #[test]
    fn mul_multiplica_dos_registros() {
        // MUL r0, r1, r2 (0xE0000291): r0 = r1·r2 = 6·7 = 42.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 6);
        cpu.set_reg(2, 7);
        cpu.execute_multiply(0xE000_0291);
        assert_eq!(cpu.reg(0), 42);
    }

    #[test]
    fn mla_multiplica_y_acumula() {
        // MLA r3, r1, r2, r4 (0xE0234291): r3 = r1·r2 + r4 = 6·7 + 100 = 142.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 6);
        cpu.set_reg(2, 7);
        cpu.set_reg(4, 100);
        cpu.execute_multiply(0xE023_4291);
        assert_eq!(cpu.reg(3), 142);
    }

    #[test]
    fn mul_se_queda_con_los_32_bits_bajos() {
        // 0x10000 · 0x10000 = 0x1_0000_0000; los 32 bits bajos son 0.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 0x0001_0000);
        cpu.set_reg(2, 0x0001_0000);
        cpu.execute_multiply(0xE000_0291); // MUL r0, r1, r2
        assert_eq!(cpu.reg(0), 0, "MUL solo conserva la palabra baja del producto");
    }

    #[test]
    fn muls_actualiza_n_y_z_y_preserva_el_carry() {
        // El ARM7TDMI deja C UNPREDECIBLE tras multiplicar; aquí lo preservamos.
        let mut cpu = Cpu::new();
        cpu.cpsr_mut().set_c(true);

        // Resultado positivo no nulo: N=0, Z=0, C intacto.
        cpu.set_reg(1, 2);
        cpu.set_reg(2, 3);
        cpu.execute_multiply(0xE010_0291); // MULS r0, r1, r2 → 6
        assert_eq!(cpu.reg(0), 6);
        assert!(!cpu.cpsr().n() && !cpu.cpsr().z());
        assert!(cpu.cpsr().c(), "C no se toca en multiply");

        // Resultado con el bit 31 a 1: N=1.
        cpu.set_reg(1, 0x8000_0000);
        cpu.set_reg(2, 1);
        cpu.execute_multiply(0xE010_0291);
        assert!(cpu.cpsr().n());

        // Resultado nulo: Z=1.
        cpu.set_reg(1, 0);
        cpu.set_reg(2, 0x1234);
        cpu.execute_multiply(0xE010_0291);
        assert!(cpu.cpsr().z());
    }

    #[test]
    fn umull_producto_sin_signo_de_64_bits() {
        // UMULL r0, r1, r2, r3 (0xE0810392): RdLo=r0, RdHi=r1, Rm=r2, Rs=r3.
        // 0xFFFFFFFF · 0xFFFFFFFF = 0xFFFFFFFE_00000001.
        let mut cpu = Cpu::new();
        cpu.set_reg(2, 0xFFFF_FFFF);
        cpu.set_reg(3, 0xFFFF_FFFF);
        cpu.execute_multiply_long(0xE081_0392);
        assert_eq!(cpu.reg(1), 0xFFFF_FFFE, "RdHi = palabra alta");
        assert_eq!(cpu.reg(0), 0x0000_0001, "RdLo = palabra baja");
    }

    #[test]
    fn smull_interpreta_los_operandos_con_signo() {
        // SMULL r0, r1, r2, r3 (0xE0C10392): con los MISMOS bits que el UMULL de
        // arriba, (-1)·(-1) = 1 → RdHi=0, RdLo=1. Es el contraste signo/sin signo.
        let mut cpu = Cpu::new();
        cpu.set_reg(2, 0xFFFF_FFFF); // -1
        cpu.set_reg(3, 0xFFFF_FFFF); // -1
        cpu.execute_multiply_long(0xE0C1_0392);
        assert_eq!(cpu.reg(1), 0);
        assert_eq!(cpu.reg(0), 1);

        // Y un caso negativo·positivo: (-3)·5 = -15 = 0xFFFFFFFF_FFFFFFF1.
        cpu.set_reg(2, (-3i32) as u32);
        cpu.set_reg(3, 5);
        cpu.execute_multiply_long(0xE0C1_0392);
        assert_eq!(cpu.reg(1), 0xFFFF_FFFF);
        assert_eq!(cpu.reg(0), (-15i32) as u32);
    }

    #[test]
    fn umlal_acumula_en_64_bits() {
        // UMLAL r0, r1, r2, r3 (0xE0A10392): RdHi:RdLo += Rm·Rs.
        // acc = 0x0000_0000_FFFF_FFFF, producto = 0x10·0x10 = 0x100.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 0x0000_0000); // RdHi (alta del acumulador)
        cpu.set_reg(0, 0xFFFF_FFFF); // RdLo (baja del acumulador)
        cpu.set_reg(2, 0x10);
        cpu.set_reg(3, 0x10);
        cpu.execute_multiply_long(0xE0A1_0392);
        // 0xFFFFFFFF + 0x100 = 0x1_0000_00FF.
        assert_eq!(cpu.reg(1), 0x0000_0001);
        assert_eq!(cpu.reg(0), 0x0000_00FF);
    }

    #[test]
    fn smlal_acumula_con_signo() {
        // SMLAL r0, r1, r2, r3 (0xE0E10392): (-1)·5 + 10 = 5 → RdHi=0, RdLo=5.
        let mut cpu = Cpu::new();
        cpu.set_reg(1, 0); // acc alto
        cpu.set_reg(0, 10); // acc bajo
        cpu.set_reg(2, 0xFFFF_FFFF); // -1
        cpu.set_reg(3, 5);
        cpu.execute_multiply_long(0xE0E1_0392);
        assert_eq!(cpu.reg(1), 0);
        assert_eq!(cpu.reg(0), 5);
    }

    #[test]
    fn umulls_actualiza_n_y_z_desde_los_64_bits() {
        // UMULLS r0, r1, r2, r3 (0xE0910392): S=1.
        let mut cpu = Cpu::new();
        // N viene del bit 63: 0xFFFFFFFF·0xFFFFFFFF = 0xFFFFFFFE_00000001.
        cpu.set_reg(2, 0xFFFF_FFFF);
        cpu.set_reg(3, 0xFFFF_FFFF);
        cpu.execute_multiply_long(0xE091_0392);
        assert!(cpu.cpsr().n(), "N = bit 63 del resultado");
        assert!(!cpu.cpsr().z());

        // Z exige que AMBAS palabras sean cero (producto nulo).
        cpu.set_reg(2, 0);
        cpu.set_reg(3, 0x1234);
        cpu.execute_multiply_long(0xE091_0392);
        assert!(cpu.cpsr().z());
        assert!(!cpu.cpsr().n());
    }

    #[test]
    fn multiply_avanza_el_pc_como_instruccion_normal() {
        // Por el step completo: MUL ya no es "no implementada"; se ejecuta, escribe
        // Rd y avanza el PC una instrucción (no es un salto).
        let base = crate::bus::IWRAM_START;
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(base, 0xE000_0291); // MUL r0, r1, r2
        let mut cpu = Cpu::new();
        cpu.set_pc(base);
        cpu.set_reg(1, 4);
        cpu.set_reg(2, 5);
        assert_eq!(cpu.step(&bus), StepResult::Stepped);
        assert_eq!(cpu.reg(0), 20);
        assert_eq!(cpu.pc(), base + 4, "no es un salto: el PC avanza 4 bytes");
    }

    #[test]
    fn mul_coste_en_ciclos_varia_con_el_multiplicador() {
        // En IWRAM (1 ciclo/fetch). MUL = 1S + mI; la S es ese fetch de 1 ciclo.
        let mut bus = Bus::new(vec![0u8; 4]);
        bus.write_u32(crate::bus::IWRAM_START, 0xE000_0291); // MUL r0, r1, r2

        // Rs = 0xFF → bits 31-8 todos cero → m=1 → 1 (fetch) + 1 = 2.
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);
        cpu.set_reg(1, 3);
        cpu.set_reg(2, 0xFF);
        cpu.step(&bus);
        assert_eq!(cpu.cycles(), 2, "multiplicador pequeño → m=1");

        // Rs = 0x12345678 → ningún byte alto homogéneo → m=4 → 1 + 4 = 5.
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);
        cpu.set_reg(1, 3);
        cpu.set_reg(2, 0x1234_5678);
        cpu.step(&bus);
        assert_eq!(cpu.cycles(), 5, "multiplicador grande → m=4");
    }

    #[test]
    fn el_coste_largo_distingue_multiplicador_con_y_sin_signo() {
        // Rs = 0xFFFFFFFF: con signo es -1 (bits altos "todo unos") → termina
        // pronto (m=1); sin signo no hay terminación por unos → m=4.
        let mut bus = Bus::new(vec![0u8; 4]);

        // SMULL r0, r1, r2, r3 (0xE0C10392): m=1 → 1S + (1+1)I = 1 + 2 = 3.
        bus.write_u32(crate::bus::IWRAM_START, 0xE0C1_0392);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);
        cpu.set_reg(2, 5);
        cpu.set_reg(3, 0xFFFF_FFFF);
        cpu.step(&bus);
        assert_eq!(cpu.cycles(), 3, "SMULL: -1 termina pronto (m=1)");

        // UMULL r0, r1, r2, r3 (0xE0810392): mismo Rs, m=4 → 1 + (4+1) = 6.
        bus.write_u32(crate::bus::IWRAM_START, 0xE081_0392);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::IWRAM_START);
        cpu.set_reg(2, 5);
        cpu.set_reg(3, 0xFFFF_FFFF);
        cpu.step(&bus);
        assert_eq!(cpu.cycles(), 6, "UMULL: 0xFFFFFFFF no termina pronto (m=4)");
    }

    #[test]
    fn multiply_internal_cycles_escalona_segun_los_bytes_altos() {
        // Con signo (allow_all_ones = true): todo ceros o todo unos termina pronto.
        assert_eq!(multiply_internal_cycles(0x0000_00FF, true), 1);
        assert_eq!(multiply_internal_cycles(0x0000_FF00, true), 2);
        assert_eq!(multiply_internal_cycles(0x00FF_0000, true), 3);
        assert_eq!(multiply_internal_cycles(0x1234_5678, true), 4);
        // "Todo unos" en los bits altos también termina pronto SOLO con signo.
        assert_eq!(multiply_internal_cycles(0xFFFF_FFFF, true), 1);
        assert_eq!(multiply_internal_cycles(0xFFFF_FFFF, false), 4);
        assert_eq!(multiply_internal_cycles(0xFF00_0000, true), 3);
        assert_eq!(multiply_internal_cycles(0xFF00_0000, false), 4);
    }
}
