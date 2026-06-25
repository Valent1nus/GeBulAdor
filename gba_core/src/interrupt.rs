//! El **sistema de interrupciones** de la GBA: los registros `IE`/`IF`/`IME` y la
//! lógica de "¿hay una IRQ que atender?" (Mini-Hito 2.3c).
//!
//! ## Cómo funciona una IRQ en la GBA
//!
//! Una *interrupt request* (IRQ) es la forma que tiene el hardware (la PPU al
//! entrar en V-Blank, un timer al desbordar, un DMA al terminar...) de avisar a la
//! CPU de que ha pasado algo, **sin** que la CPU tenga que estar preguntando en un
//! bucle. Tres registros la gobiernan:
//!
//! - **`IE`** (*Interrupt Enable*, `0x0400_0200`, 16 bits): un bit por fuente
//!   ([`Interrupt`]). A 1 = "quiero que esta fuente me interrumpa".
//! - **`IF`** (*Interrupt Flags*, `0x0400_0202`, 16 bits): los avisos
//!   **pendientes**. El hardware pone un bit cuando su fuente dispara. El software
//!   lo **reconoce** (*acknowledge*) escribiendo un **1** en ese bit, lo que lo
//!   **borra** (la peculiaridad "write-1-to-clear": escribir 0 no hace nada).
//! - **`IME`** (*Interrupt Master Enable*, `0x0400_0208`, bit 0): el interruptor
//!   general. Con `IME = 0`, ninguna IRQ se atiende aunque `IE & IF` sea distinto
//!   de cero.
//!
//! La CPU atiende una IRQ cuando se cumplen **tres** condiciones a la vez:
//! `IME = 1`, `(IE & IF) != 0` y el bit `I` del `CPSR` está a 0 (las dos primeras
//! las resuelve [`InterruptControl::pending`]; la del `CPSR` la comprueba la CPU).
//! Entonces salta al **vector `0x18`** en modo IRQ (ver [`crate::Cpu`]).
//!
//! ## Reparto con el [`crate::Bus`]
//!
//! Este módulo es la **fuente de verdad** de `IE`/`IF`/`IME`. El bus enruta aquí
//! sus accesos (como hace con el [`crate::dma`]) y ofrece a los componentes
//! ([`crate::Bus::request_interrupt`]) y a la CPU
//! ([`crate::Bus::irq_pending`]/[`crate::Bus::irq_raised`]) la API de alto nivel.

/// Las 14 **fuentes de interrupción** de la GBA, en el orden de bit que ocupan en
/// `IE`/`IF` (GBATEK). El número de la variante **es** su número de bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Interrupt {
    /// LCD V-Blank: la PPU terminó de dibujar las 160 líneas visibles.
    VBlank = 0,
    /// LCD H-Blank: la PPU terminó una línea.
    HBlank = 1,
    /// LCD V-Counter: la línea actual coincide con el valor programado.
    VCounter = 2,
    /// Desbordamiento del Timer 0.
    Timer0 = 3,
    /// Desbordamiento del Timer 1.
    Timer1 = 4,
    /// Desbordamiento del Timer 2.
    Timer2 = 5,
    /// Desbordamiento del Timer 3.
    Timer3 = 6,
    /// Comunicación serie (Cable Link, Fase 4).
    Serial = 7,
    /// Fin de transferencia del DMA 0.
    Dma0 = 8,
    /// Fin de transferencia del DMA 1.
    Dma1 = 9,
    /// Fin de transferencia del DMA 2.
    Dma2 = 10,
    /// Fin de transferencia del DMA 3.
    Dma3 = 11,
    /// Teclado (una combinación de teclas programada).
    Keypad = 12,
    /// Hardware externo del cartucho (Game Pak).
    GamePak = 13,
}

impl Interrupt {
    /// La máscara de un solo bit que esta fuente ocupa en `IE`/`IF`.
    pub fn bit(self) -> u16 {
        1 << (self as u16)
    }

    /// La fuente de IRQ del canal de DMA `channel` (0–3). Un `channel` fuera de
    /// rango se trata como DMA3 (defensa: el llamante siempre pasa 0–3).
    pub fn dma(channel: usize) -> Interrupt {
        match channel {
            0 => Interrupt::Dma0,
            1 => Interrupt::Dma1,
            2 => Interrupt::Dma2,
            _ => Interrupt::Dma3,
        }
    }

    /// La fuente de IRQ del timer `index` (0–3). Un `index` fuera de rango se trata
    /// como Timer3 (defensa: el llamante siempre pasa 0–3).
    pub fn timer(index: usize) -> Interrupt {
        match index {
            0 => Interrupt::Timer0,
            1 => Interrupt::Timer1,
            2 => Interrupt::Timer2,
            _ => Interrupt::Timer3,
        }
    }
}

// Offsets (dentro de la región de I/O, base `0x0400_0000`) de los registros.
/// `IE` (16 bits): los dos bytes en `0x200`–`0x201`.
const IE_LO: u32 = 0x200;
/// `IF` (16 bits): los dos bytes en `0x202`–`0x203`.
const IF_LO: u32 = 0x202;
/// `IME` (32 bits; solo el bit 0 es útil): bytes en `0x208`–`0x20B`.
const IME_LO: u32 = 0x208;

/// Bits válidos de `IE`/`IF`: las 14 fuentes (0–13). Los bits 14-15 no existen.
const IRQ_MASK: u16 = 0x3FFF;

/// El controlador de interrupciones: `IE`, `IF` e `IME`.
///
/// Vive **dentro** del [`crate::Bus`], que enruta aquí los accesos a sus
/// registros. Es la fuente de verdad de las tres palabras de estado.
pub struct InterruptControl {
    /// `IE`: máscara de fuentes habilitadas (bit por [`Interrupt`]).
    ie: u16,
    /// `IF`: avisos pendientes. Los pone el hardware ([`InterruptControl::request`])
    /// y los borra el software escribiendo un 1 (*write-1-to-clear*).
    requested: u16,
    /// `IME`: interruptor maestro. `false` = ninguna IRQ se atiende.
    ime: bool,
}

impl InterruptControl {
    /// Crea el controlador en reposo: todo deshabilitado y sin avisos pendientes
    /// (el estado de reset del hardware).
    pub fn new() -> Self {
        InterruptControl {
            ie: 0,
            requested: 0,
            ime: false,
        }
    }

    /// `true` si el offset de I/O `io_off` (los 24 bits bajos de la dirección)
    /// corresponde a `IE`, `IF` o `IME`. Lo usa el bus para enrutar aquí el acceso.
    ///
    /// Deja fuera a propósito `0x204`–`0x207` (`WAITCNT`, que aún no tiene
    /// semántica propia y sigue en el buffer de I/O crudo).
    pub fn handles(io_off: u32) -> bool {
        (IE_LO..IF_LO + 2).contains(&io_off) || (IME_LO..IME_LO + 4).contains(&io_off)
    }

    /// Lee un byte de `IE`/`IF`/`IME`. Los bytes no usados (`IME` bits 8+) leen 0.
    pub fn read_u8(&self, io_off: u32) -> u8 {
        match io_off {
            IE_LO => self.ie as u8,
            o if o == IE_LO + 1 => (self.ie >> 8) as u8,
            IF_LO => self.requested as u8,
            o if o == IF_LO + 1 => (self.requested >> 8) as u8,
            IME_LO => self.ime as u8,
            _ => 0,
        }
    }

    /// Escribe un byte en `IE`/`IF`/`IME`, respetando la semántica de cada uno:
    /// `IE`/`IME` se asignan, pero un byte de `IF` **borra** (no asigna) los bits
    /// puestos a 1 (*write-1-to-clear*, el *acknowledge* del manejador de IRQ).
    pub fn write_u8(&mut self, io_off: u32, value: u8) {
        match io_off {
            IE_LO => self.ie = (self.ie & 0xFF00) | u16::from(value),
            o if o == IE_LO + 1 => self.ie = (self.ie & 0x00FF) | (u16::from(value) << 8),
            // IF: escribir un 1 reconoce (borra) ese aviso pendiente.
            IF_LO => self.requested &= !u16::from(value),
            o if o == IF_LO + 1 => self.requested &= !(u16::from(value) << 8),
            IME_LO => self.ime = value & 1 != 0,
            _ => {}
        }
    }

    /// Marca la IRQ de `source` como **pendiente** (pone su bit en `IF`). La llaman
    /// los componentes de hardware al disparar (DMA al terminar, timer al
    /// desbordar...), vía [`crate::Bus::request_interrupt`].
    pub fn request(&mut self, source: Interrupt) {
        self.requested |= source.bit();
    }

    /// `true` si la fuente `source` está **habilitada** en `IE`. Lo usa el bus para
    /// saber si un timer con IRQ podría despertar a la CPU de un `Halt` (2.3e).
    pub fn is_enabled(&self, source: Interrupt) -> bool {
        self.ie & source.bit() != 0
    }

    /// `true` si hay **alguna IRQ habilitada y pendiente** (`IE & IF != 0`), **sin**
    /// mirar `IME`. Es la condición que despierta a la CPU de un `Halt` (que no
    /// depende de `IME`, según GBATEK).
    pub fn raised(&self) -> bool {
        self.ie & self.requested & IRQ_MASK != 0
    }

    /// `true` si una IRQ debe **atenderse**: lo de [`InterruptControl::raised`] y,
    /// además, `IME = 1`. (La CPU comprueba aparte el bit `I` del `CPSR`.)
    pub fn pending(&self) -> bool {
        self.ime && self.raised()
    }
}

impl Default for InterruptControl {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handles_reconoce_ie_if_ime_pero_no_waitcnt() {
        assert!(InterruptControl::handles(0x200)); // IE
        assert!(InterruptControl::handles(0x201));
        assert!(InterruptControl::handles(0x202)); // IF
        assert!(InterruptControl::handles(0x203));
        assert!(!InterruptControl::handles(0x204)); // WAITCNT: no es nuestro
        assert!(!InterruptControl::handles(0x207));
        assert!(InterruptControl::handles(0x208)); // IME
        assert!(InterruptControl::handles(0x20B));
        assert!(!InterruptControl::handles(0x20C));
    }

    #[test]
    fn ie_e_ime_se_leen_como_se_escriben() {
        let mut irq = InterruptControl::new();
        irq.write_u8(0x200, 0x34);
        irq.write_u8(0x201, 0x12);
        assert_eq!(irq.read_u8(0x200), 0x34);
        assert_eq!(irq.read_u8(0x201), 0x12);
        irq.write_u8(0x208, 1);
        assert_eq!(irq.read_u8(0x208), 1);
        // El bit 0 es lo único de IME: escribir un valor par lo apaga.
        irq.write_u8(0x208, 0xFE);
        assert_eq!(irq.read_u8(0x208), 0);
    }

    #[test]
    fn if_se_borra_escribiendo_un_uno() {
        let mut irq = InterruptControl::new();
        irq.request(Interrupt::VBlank); // bit 0
        irq.request(Interrupt::Timer0); // bit 3
        assert_eq!(irq.read_u8(0x202) & 0b1001, 0b1001);
        // Reconocer (acknowledge) solo el V-Blank: escribir un 1 en su bit lo borra.
        irq.write_u8(0x202, 0x01);
        assert_eq!(irq.read_u8(0x202) & 0x01, 0, "V-Blank reconocido (borrado)");
        assert_ne!(irq.read_u8(0x202) & 0x08, 0, "Timer0 sigue pendiente");
        // Escribir un 0 no borra nada.
        irq.write_u8(0x202, 0x00);
        assert_ne!(irq.read_u8(0x202) & 0x08, 0);
    }

    #[test]
    fn raised_y_pending_combinan_ie_if_e_ime() {
        let mut irq = InterruptControl::new();
        irq.request(Interrupt::Dma0); // IF bit 8
        assert!(!irq.raised(), "sin IE habilitado, no cuenta");
        // Habilitar el DMA0 en IE (bit 8).
        irq.write_u8(0x201, 0x01); // byte alto de IE = 0x01 → bit 8
        assert!(irq.raised(), "IE & IF != 0");
        assert!(!irq.pending(), "pero IME está a 0");
        irq.write_u8(0x208, 1); // IME = 1
        assert!(irq.pending(), "ahora sí se atiende");
    }

    #[test]
    fn la_fuente_dma_mapea_al_bit_correcto() {
        assert_eq!(Interrupt::dma(0), Interrupt::Dma0);
        assert_eq!(Interrupt::dma(3), Interrupt::Dma3);
        assert_eq!(Interrupt::Dma0.bit(), 1 << 8);
        assert_eq!(Interrupt::VBlank.bit(), 1 << 0);
        assert_eq!(Interrupt::GamePak.bit(), 1 << 13);
    }
}
