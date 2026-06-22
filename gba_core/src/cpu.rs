//! La CPU **ARM7TDMI** de la Game Boy Advance: registros, estado y modos.
//!
//! Este módulo solo construye el *esqueleto* donde más adelante vivirá la
//! ejecución de instrucciones (Mini-Hitos 2.1b en adelante). De momento modela:
//!
//! - Los **16 registros visibles** `r0`–`r15` (`r13` = SP, `r14` = LR,
//!   `r15` = PC).
//! - El registro de estado **CPSR** (flags de condición + bits de control).
//! - Los **modos de operación** del procesador y, lo más importante, el
//!   *banking* de registros entre modos.
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

use crate::bus::Bus;

/// Número de registros visibles del ARM7TDMI: `r0`–`r15`.
pub const NUM_REGISTERS: usize = 16;

/// Índice del *Stack Pointer* (`r13`).
pub const SP: usize = 13;
/// Índice del *Link Register* (`r14`), donde `BL` deja la dirección de retorno.
pub const LR: usize = 14;
/// Índice del *Program Counter* (`r15`).
pub const PC: usize = 15;

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

    /// Lee un registro visible por índice (`0`–`15`).
    ///
    /// El índice siempre proviene de un campo de 4 bits de una instrucción ya
    /// decodificada, así que está garantizado en rango; el `debug_assert!` lo
    /// verifica en builds de depuración sin coste en release.
    ///
    /// *(Nota para el futuro: en el Mini-Hito 2.1e, leer `r15` deberá devolver el
    /// `PC` con el desfase del pipeline aplicado (+8 en ARM, +4 en THUMB). Por
    /// ahora devolvemos el valor crudo.)*
    pub fn reg(&self, index: usize) -> u32 {
        debug_assert!(index < NUM_REGISTERS, "índice de registro fuera de rango: {index}");
        self.r[index]
    }

    /// Escribe un registro visible por índice (`0`–`15`).
    pub fn set_reg(&mut self, index: usize, value: u32) {
        debug_assert!(index < NUM_REGISTERS, "índice de registro fuera de rango: {index}");
        self.r[index] = value;
    }

    /// El Program Counter (`r15`).
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

    /// **Fetch**: lee la instrucción ARM (32 bits) a la que apunta el `PC`, en
    /// little-endian, a través del bus. Es la primera etapa del ciclo
    /// Fetch→Decode→Execute (Mini-Hito 2.1b).
    ///
    /// No avanza ni modifica el `PC`: es una lectura pura. El avance del puntero
    /// llega con el bucle de ejecución (2.2a) y el desfase de pipeline con el
    /// 2.1e.
    ///
    /// De momento siempre lee 4 bytes (modo ARM). El Mini-Hito 2.3a añadirá la
    /// rama THUMB: leer 2 bytes cuando el bit `T` del CPSR esté activo.
    pub fn fetch(&self, bus: &Bus) -> u32 {
        bus.read_u32(self.pc())
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
        for i in 0..NUM_REGISTERS {
            assert_eq!(cpu.reg(i), 0);
        }
    }

    #[test]
    fn lee_y_escribe_registros() {
        let mut cpu = Cpu::new();
        cpu.set_reg(0, 0xDEAD_BEEF);
        cpu.set_pc(0x0800_0000);
        assert_eq!(cpu.reg(0), 0xDEAD_BEEF);
        assert_eq!(cpu.pc(), 0x0800_0000);
        assert_eq!(cpu.reg(PC), 0x0800_0000);
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
}
