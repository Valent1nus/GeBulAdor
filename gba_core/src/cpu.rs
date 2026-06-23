//! La CPU **ARM7TDMI** de la Game Boy Advance: registros, estado y modos.
//!
//! Este módulo modela el estado de la CPU y, sobre él, el ciclo
//! Fetch→Decode→Execute tal como está implementado hasta el Mini-Hito 2.1e. De
//! momento cubre:
//!
//! - Los **16 registros visibles** `r0`–`r15` (`r13` = SP, `r14` = LR,
//!   `r15` = PC).
//! - El registro de estado **CPSR** (flags de condición + bits de control).
//! - Los **modos de operación** del procesador y, lo más importante, el
//!   *banking* de registros entre modos.
//! - El **Fetch** ([`Cpu::fetch`]), el **Decode** de ARM ([`Cpu::decode_arm`]) y
//!   THUMB ([`Cpu::decode_thumb`]), y el inicio del **Execute**
//!   ([`Cpu::execute_data_processing`]: la primera familia de instrucciones, el
//!   procesamiento de datos con operando inmediato).
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
use crate::bus::Bus;
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
}

/// Informe de una corrida en bucle ([`Cpu::run`] / [`crate::Gba::run`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunReport {
    /// Instrucciones ejecutadas antes de parar (sin contar la que provocó la
    /// parada, que no llega a ejecutarse).
    pub steps: u64,
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

    /// Ejecuta una instrucción de **procesamiento de datos** en su forma con
    /// operando inmediato (Mini-Hito 2.1d). Es la primera instrucción que altera
    /// el estado de la CPU: calcula el resultado de la operación de la ALU
    /// (`MOV`, `ADD`, `SUB`, `AND`...), lo escribe en `Rd` salvo para las
    /// comparaciones (`TST`/`TEQ`/`CMP`/`CMN`) y actualiza los flags `N/Z/C/V` si
    /// el bit `S` (20) está activo.
    ///
    /// Se asume que la condición de la instrucción ya se evaluó (vía
    /// [`Cpu::decode_arm`]) y se cumple. **Solo** está implementada la forma con
    /// operando inmediato; la forma con registro desplazado (*barrel shifter*) y
    /// los casos especiales con `Rd = r15` (PC) llegan en hitos posteriores.
    pub fn execute_data_processing(&mut self, instr: u32) {
        // Capturamos el CPSR de ENTRADA: el carry previo lo necesitan ADC/SBC y
        // el carry del shifter, y debe leerse antes de actualizar los flags.
        let cpsr_before = self.cpsr();

        // --- Operando 2 (solo forma inmediata, bit 25 = 1) ----------------
        // Un valor de 8 bits rotado a la derecha por un campo de 4 bits (×2).
        let is_immediate = (instr & (1 << 25)) != 0;
        debug_assert!(
            is_immediate,
            "data-processing con operando de registro aún no implementado (barrel shifter)"
        );
        if !is_immediate {
            return;
        }
        let rotate = ((instr >> 8) & 0xF) * 2;
        let imm8 = instr & 0xFF;
        let operand2 = imm8.rotate_right(rotate);
        // Carry del shifter (para las operaciones lógicas): con rotación 0 se
        // conserva el C actual; con rotación, es el bit 31 del resultado.
        let shifter_carry = if rotate == 0 {
            cpsr_before.c()
        } else {
            (operand2 & 0x8000_0000) != 0
        };

        // --- Operandos y opcode -------------------------------------------
        let rn = ((instr >> 16) & 0xF) as usize;
        let rd = ((instr >> 12) & 0xF) as usize;
        let opcode = (instr >> 21) & 0xF;
        let sets_flags = (instr & (1 << 20)) != 0;
        let a = self.reg(rn);
        let b = operand2;
        let carry_in = cpsr_before.c();

        // --- Operación de la ALU ------------------------------------------
        // Las lógicas dejan V sin tocar (`None`) y usan el carry del shifter;
        // las aritméticas obtienen carry/overflow de la suma. La resta se modela
        // como `a + !b + 1`, así que [`alu_add`] cubre todos los casos.
        let (result, carry, overflow): (u32, bool, Option<bool>) = match opcode {
            0x0 => (a & b, shifter_carry, None),    // AND
            0x1 => (a ^ b, shifter_carry, None),    // EOR
            0x2 => with_v(alu_add(a, !b, true)),    // SUB
            0x3 => with_v(alu_add(b, !a, true)),    // RSB
            0x4 => with_v(alu_add(a, b, false)),    // ADD
            0x5 => with_v(alu_add(a, b, carry_in)), // ADC
            0x6 => with_v(alu_add(a, !b, carry_in)), // SBC
            0x7 => with_v(alu_add(b, !a, carry_in)), // RSC
            0x8 => (a & b, shifter_carry, None),    // TST
            0x9 => (a ^ b, shifter_carry, None),    // TEQ
            0xA => with_v(alu_add(a, !b, true)),    // CMP
            0xB => with_v(alu_add(a, b, false)),    // CMN
            0xC => (a | b, shifter_carry, None),    // ORR
            0xD => (b, shifter_carry, None),        // MOV
            0xE => (a & !b, shifter_carry, None),   // BIC
            0xF => (!b, shifter_carry, None),       // MVN
            _ => unreachable!("opcode = (instr >> 21) & 0xF está en 0..=15"),
        };

        // --- Flags (solo si S = 1) ----------------------------------------
        if sets_flags {
            let cpsr = self.cpsr_mut();
            cpsr.set_n((result & 0x8000_0000) != 0);
            cpsr.set_z(result == 0);
            cpsr.set_c(carry);
            if let Some(v) = overflow {
                cpsr.set_v(v);
            }
        }

        // --- Escritura del resultado --------------------------------------
        // Las comparaciones (TST/TEQ/CMP/CMN, opcodes 0x8..=0xB) no escriben Rd.
        if !matches!(opcode, 0x8..=0xB) {
            self.set_reg(rd, result);
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

    /// Ejecuta **un paso**: fetch → decode → execute de una sola instrucción,
    /// avanzando el `PC` si procede (Mini-Hito 2.2a).
    ///
    /// Hoy solo se decodifica/ejecuta el set **ARM** (el fetch THUMB de 2 bytes
    /// llega en el 2.3a) y, dentro de ARM, solo el procesamiento de datos con
    /// operando inmediato. Ante cualquier otra instrucción la CPU se detiene
    /// limpiamente con [`StepResult::Halted`], **sin** avanzar el `PC` (queda
    /// apuntando a la instrucción culpable para poder inspeccionarla).
    pub fn step(&mut self, bus: &Bus) -> StepResult {
        let pc = self.pc();
        let instr = self.fetch(bus);

        match self.decode_arm(instr) {
            // Condición no cumplida: la instrucción es un NOP de un ciclo. Lo
            // único que hace es dejar pasar el tiempo, así que solo avanzamos.
            Decoded::ConditionFailed(_) => {
                self.advance_pc();
                StepResult::Stepped
            }
            Decoded::Execute(kind) => {
                if self.try_execute_arm(kind, instr) {
                    // Las instrucciones soportadas hoy nunca escriben el `PC`, así
                    // que tras ejecutarlas se pasa a la siguiente.
                    self.advance_pc();
                    StepResult::Stepped
                } else if is_branch_to_self(kind, instr, pc) {
                    // Un «b .» (salto a su propia dirección) es un bucle infinito
                    // de una instrucción: no hace falta ejecutarlo para saber que
                    // no avanza. Es la señal de "fin" de las ROMs de test (2.2b).
                    StepResult::Halted(Halt::InfiniteLoop { pc, instr })
                } else {
                    StepResult::Halted(Halt::Unimplemented { pc, instr, kind })
                }
            }
        }
    }

    /// Ejecuta pasos en bucle hasta que la CPU se detiene ([`StepResult::Halted`])
    /// o hasta completar `max_steps` instrucciones (Mini-Hito 2.2a).
    ///
    /// El tope `max_steps` es una **salvaguarda**: mientras falten instrucciones
    /// por implementar, una secuencia de NOPs (p. ej. memoria a cero) avanzaría
    /// el `PC` indefinidamente; sin un límite, el bucle no terminaría nunca.
    pub fn run(&mut self, bus: &Bus, max_steps: u64) -> RunReport {
        let mut steps = 0;
        while steps < max_steps {
            match self.step(bus) {
                StepResult::Stepped => steps += 1,
                StepResult::Halted(halt) => {
                    return RunReport {
                        steps,
                        stop: RunStop::Halted(halt),
                    };
                }
            }
        }
        RunReport {
            steps,
            stop: RunStop::StepLimit,
        }
    }

    /// Intenta ejecutar la instrucción ARM `kind` (bits crudos en `instr`),
    /// asumiendo que su condición ya pasó. Devuelve `true` si la ejecutó, o
    /// `false` si esa instrucción —o esa variante— aún no está implementada (lo
    /// que hará que [`Cpu::step`] detenga el bucle).
    ///
    /// A medida que se implementen instrucciones, este `match` ganará ramas.
    fn try_execute_arm(&mut self, kind: ArmInstruction, instr: u32) -> bool {
        match kind {
            ArmInstruction::DataProcessing => {
                // Solo la forma con operando inmediato (bit 25) y destino distinto
                // de `r15`. La forma con registro desplazado (barrel shifter) y
                // escribir el `PC` (un salto) llegan más adelante; hasta entonces
                // se tratan como no implementadas en vez de ejecutarse a medias.
                let is_immediate = (instr & (1 << 25)) != 0;
                let rd = (instr >> 12) & 0xF;
                if is_immediate && rd != PC as u32 {
                    self.execute_data_processing(instr);
                    true
                } else {
                    false
                }
            }
            _ => false,
        }
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
        // MOV r0,#5 ; ADD r0,r0,#1 ; B (salto, aún sin ejecutar).
        let programa = [0xE3A0_0005u32, 0xE280_0001, 0xEA00_0000];
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

        // La tercera es un salto: no implementado → la CPU se detiene en él.
        let pc_culpable = cpu.pc();
        match cpu.step(&bus) {
            StepResult::Halted(Halt::Unimplemented { pc, instr, kind }) => {
                assert_eq!(pc, crate::bus::ROM_START + 8);
                assert_eq!(instr, 0xEA00_0000);
                assert_eq!(kind, ArmInstruction::Branch { link: false });
            }
            otro => panic!("esperaba Halted, fue {otro:?}"),
        }
        // El PC NO avanzó: sigue apuntando a la instrucción no implementada.
        assert_eq!(cpu.pc(), pc_culpable);
    }

    #[test]
    fn data_processing_con_registro_aun_no_se_ejecuta() {
        // MOV r0, r1 (forma con registro, bit 25 = 0): 0xE1A00001. Todavía no
        // implementada → el paso se detiene en vez de "ejecutar" en silencio.
        let mut rom = vec![0u8; 8];
        rom[0..4].copy_from_slice(&0xE1A0_0001u32.to_le_bytes());
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);
        assert!(matches!(cpu.step(&bus), StepResult::Halted(_)));
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
    fn un_salto_que_no_es_a_si_mismo_sigue_siendo_no_implementado() {
        // 0xEA00002E = salto hacia delante (el de arranque de las ROMs reales):
        // NO es un bucle a sí mismo, así que se reporta como no implementado
        // hasta el Mini-Hito 2.2e.
        let mut rom = vec![0u8; 8];
        rom[0..4].copy_from_slice(&0xEA00_002Eu32.to_le_bytes());
        let bus = Bus::new(rom);
        let mut cpu = Cpu::new();
        cpu.set_pc(crate::bus::ROM_START);
        assert!(matches!(
            cpu.step(&bus),
            StepResult::Halted(Halt::Unimplemented { .. })
        ));
    }
}
