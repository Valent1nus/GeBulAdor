//! Decodificación de instrucciones en **modo ARM** (32 bits).
//!
//! Este módulo implementa el Mini-Hito 2.1c: identificar *qué tipo* de
//! instrucción es una palabra de 32 bits, **sin ejecutar todavía su lógica**.
//!
//! ## ⚠️ El decode ARM es en DOS pasos (no uno)
//!
//! La trampa que casi todo el mundo comete al empezar es meter los 32 bits en un
//! único `match` gigante. No funciona, porque los **bits 31-28 de TODA
//! instrucción ARM son un código de condición** ([`Condition`]) — no parte del
//! opcode. La mayoría del código real de los juegos es condicional (`MOVEQ`,
//! `BNE`, `ADDLT`...), así que si no separas la condición del opcode desde el
//! principio, el `match` se vuelve inmanejable.
//!
//! El flujo correcto, que modela [`decode`], es:
//!
//! 1. **Extraer los bits 31-28** y evaluar la condición contra el CPSR actual.
//! 2. Si **no** se cumple → la instrucción se descarta (actúa como un NOP de un
//!    ciclo): [`Decoded::ConditionFailed`].
//! 3. Si **sí** se cumple → solo entonces se miran los bits 27-0 para clasificar
//!    el opcode ([`ArmInstruction::decode`]): [`Decoded::Execute`].
//!
//! ## Por qué la clasificación necesita orden (no es un `match` plano)
//!
//! El encoding ARMv4T tiene formatos que **se solapan** en los bits altos: por
//! ejemplo, `MUL`, `SWP`, `BX` y los accesos a media palabra viven todos en el
//! mismo espacio que el procesamiento de datos (bits 27-26 = `00`) y solo se
//! distinguen por bits bajos (7-4). Por eso [`ArmInstruction::decode`] comprueba
//! primero esos casos especiales con máscaras concretas y, solo después, cae al
//! `match` por la categoría principal (bits 27-25).

use std::fmt;

use crate::cpu::Cpsr;

/// Los 16 **códigos de condición** ARM (bits 31-28 de toda instrucción).
///
/// El valor de cada variante es exactamente el patrón de 4 bits del campo en la
/// instrucción, lo que documenta el encoding y permite el `#[repr(u8)]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Condition {
    /// Igual (`Z = 1`).
    Eq = 0x0,
    /// Distinto (`Z = 0`).
    Ne = 0x1,
    /// Carry activo / mayor-o-igual sin signo (`C = 1`). También `HS`.
    Cs = 0x2,
    /// Carry inactivo / menor sin signo (`C = 0`). También `LO`.
    Cc = 0x3,
    /// Negativo (`N = 1`).
    Mi = 0x4,
    /// Positivo o cero (`N = 0`).
    Pl = 0x5,
    /// Desbordamiento (`V = 1`).
    Vs = 0x6,
    /// Sin desbordamiento (`V = 0`).
    Vc = 0x7,
    /// Mayor sin signo (`C = 1` y `Z = 0`).
    Hi = 0x8,
    /// Menor-o-igual sin signo (`C = 0` o `Z = 1`).
    Ls = 0x9,
    /// Mayor-o-igual con signo (`N = V`).
    Ge = 0xA,
    /// Menor con signo (`N != V`).
    Lt = 0xB,
    /// Mayor con signo (`Z = 0` y `N = V`).
    Gt = 0xC,
    /// Menor-o-igual con signo (`Z = 1` o `N != V`).
    Le = 0xD,
    /// Siempre (la condición por defecto del código no condicional).
    Al = 0xE,
    /// Nunca (reservado en ARMv4T; nunca se ejecuta).
    Nv = 0xF,
}

impl Condition {
    /// Extrae el código de condición de una instrucción (bits 31-28). Paso 1 del
    /// decode. Como el campo son 4 bits, siempre corresponde a una de las 16
    /// variantes (no hay caso inválido).
    pub fn from_instr(instr: u32) -> Condition {
        match (instr >> 28) & 0xF {
            0x0 => Condition::Eq,
            0x1 => Condition::Ne,
            0x2 => Condition::Cs,
            0x3 => Condition::Cc,
            0x4 => Condition::Mi,
            0x5 => Condition::Pl,
            0x6 => Condition::Vs,
            0x7 => Condition::Vc,
            0x8 => Condition::Hi,
            0x9 => Condition::Ls,
            0xA => Condition::Ge,
            0xB => Condition::Lt,
            0xC => Condition::Gt,
            0xD => Condition::Le,
            0xE => Condition::Al,
            0xF => Condition::Nv,
            _ => unreachable!("(instr >> 28) & 0xF es un nibble de 4 bits"),
        }
    }

    /// `true` si la condición se cumple con los flags `N/Z/C/V` del `cpsr` dado.
    pub fn passes(self, cpsr: Cpsr) -> bool {
        let (n, z, c, v) = (cpsr.n(), cpsr.z(), cpsr.c(), cpsr.v());
        match self {
            Condition::Eq => z,
            Condition::Ne => !z,
            Condition::Cs => c,
            Condition::Cc => !c,
            Condition::Mi => n,
            Condition::Pl => !n,
            Condition::Vs => v,
            Condition::Vc => !v,
            Condition::Hi => c && !z,
            Condition::Ls => !c || z,
            Condition::Ge => n == v,
            Condition::Lt => n != v,
            Condition::Gt => !z && (n == v),
            Condition::Le => z || (n != v),
            Condition::Al => true,
            Condition::Nv => false,
        }
    }
}

/// Categoría (formato) de una instrucción ARM, tal como la identifica el decode.
///
/// De momento solo se **clasifica**: ninguna variante lleva todavía los campos
/// descodificados (registros, inmediatos...) ni lógica de ejecución; eso empieza
/// en el Mini-Hito 2.1d.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmInstruction {
    /// Procesamiento de datos: `ADD`, `SUB`, `MOV`, `AND`, `ORR`, `CMP`...
    DataProcessing,
    /// Transferencia con el registro de estado: `MRS` / `MSR`.
    PsrTransfer,
    /// Multiplicación de 32 bits: `MUL` / `MLA`.
    Multiply,
    /// Multiplicación de 64 bits: `UMULL` / `UMLAL` / `SMULL` / `SMLAL`.
    MultiplyLong,
    /// Intercambio atómico registro↔memoria: `SWP` / `SWPB`.
    SingleDataSwap,
    /// Salto e intercambio de estado ARM/THUMB: `BX`.
    BranchExchange,
    /// Carga/almacén de media palabra o byte con signo: `LDRH`/`STRH`/`LDRSB`/`LDRSH`.
    HalfwordTransfer,
    /// Carga/almacén de una palabra o byte: `LDR` / `STR`.
    SingleDataTransfer,
    /// Carga/almacén en bloque (varios registros): `LDM` / `STM`.
    BlockDataTransfer,
    /// Salto relativo: `B` (`link == false`) o `BL` (`link == true`).
    Branch {
        /// `true` si guarda la dirección de retorno en `LR` (`BL`).
        link: bool,
    },
    /// Operación de coprocesador (`CDP`/`LDC`/`STC`/`MRC`/`MCR`). La GBA no tiene
    /// coprocesadores, así que en la práctica no aparece en ROMs reales.
    Coprocessor,
    /// Interrupción software: `SWI` (en la GBA, las llamadas a la BIOS).
    SoftwareInterrupt,
    /// Patrón de bits que no corresponde a ninguna instrucción válida.
    Undefined,
}

impl ArmInstruction {
    /// Clasifica el opcode de una instrucción ARM (paso 2 del decode).
    ///
    /// **No** mira la condición (bits 31-28): eso es trabajo de [`Condition`]. El
    /// orden de las comprobaciones importa, porque varios formatos comparten los
    /// bits 27-26 = `00` y solo se distinguen por bits bajos (ver la cabecera del
    /// módulo).
    pub fn decode(instr: u32) -> ArmInstruction {
        // Categoría principal: bits 27-25.
        let op = (instr >> 25) & 0x7;

        // --- Casos especiales del espacio "00x", por máscara concreta ---------

        // `BX`: cccc 0001 0010 1111 1111 1111 0001 nnnn
        if (instr & 0x0FFF_FFF0) == 0x012F_FF10 {
            return ArmInstruction::BranchExchange;
        }
        // `MUL`/`MLA`: bits 27-22 = 000000 y bits 7-4 = 1001.
        if (instr & 0x0FC0_00F0) == 0x0000_0090 {
            return ArmInstruction::Multiply;
        }
        // `UMULL`/...: bits 27-23 = 00001 y bits 7-4 = 1001.
        if (instr & 0x0F80_00F0) == 0x0080_0090 {
            return ArmInstruction::MultiplyLong;
        }
        // `SWP`: bits 27-23 = 00010, bits 21-20 = 00, bits 11-4 = 0000 1001.
        if (instr & 0x0FB0_0FF0) == 0x0100_0090 {
            return ArmInstruction::SingleDataSwap;
        }
        // Media palabra / byte con signo: espacio 00x, bit 7 = 1, bit 4 = 1 y
        // bits 6-5 != 00 (si fueran 00 sería MUL/SWP, ya capturados arriba).
        if op == 0b000 && (instr & 0x90) == 0x90 && (instr & 0x60) != 0 {
            return ArmInstruction::HalfwordTransfer;
        }

        // --- Categoría principal por bits 27-25 -------------------------------
        match op {
            0b000 | 0b001 => {
                // Procesamiento de datos... salvo el truco de `MRS`/`MSR`: un
                // opcode de comparación (TST/TEQ/CMP/CMN, bits 24-21 = 10xx) con
                // el bit S (20) a 0 no tiene sentido como comparación, así que el
                // hardware lo reutiliza para la transferencia de PSR.
                let opcode = (instr >> 21) & 0xF;
                let sets_flags = (instr & (1 << 20)) != 0;
                if !sets_flags && (opcode & 0b1100) == 0b1000 {
                    ArmInstruction::PsrTransfer
                } else {
                    ArmInstruction::DataProcessing
                }
            }
            0b010 => ArmInstruction::SingleDataTransfer,
            0b011 => {
                // `LDR`/`STR` con offset de registro; el bit 4 a 1 aquí está
                // indefinido en ARMv4T.
                if (instr & (1 << 4)) != 0 {
                    ArmInstruction::Undefined
                } else {
                    ArmInstruction::SingleDataTransfer
                }
            }
            0b100 => ArmInstruction::BlockDataTransfer,
            0b101 => ArmInstruction::Branch {
                link: (instr & (1 << 24)) != 0,
            },
            0b110 => ArmInstruction::Coprocessor,
            0b111 => {
                if (instr & (1 << 24)) != 0 {
                    ArmInstruction::SoftwareInterrupt
                } else {
                    ArmInstruction::Coprocessor
                }
            }
            _ => unreachable!("op = (instr >> 25) & 0x7 está en 0..=7"),
        }
    }
}

impl fmt::Display for ArmInstruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            ArmInstruction::Branch { link: false } => "Salto (B / Branch)",
            ArmInstruction::Branch { link: true } => "Salto con enlace (BL / Branch with Link)",
            ArmInstruction::BranchExchange => "Salto e intercambio de estado (BX)",
            ArmInstruction::DataProcessing => "Procesamiento de datos (ADD, SUB, MOV, ...)",
            ArmInstruction::PsrTransfer => "Transferencia de PSR (MRS / MSR)",
            ArmInstruction::Multiply => "Multiplicación (MUL / MLA)",
            ArmInstruction::MultiplyLong => "Multiplicación larga (UMULL / SMULL / ...)",
            ArmInstruction::SingleDataSwap => "Intercambio con memoria (SWP)",
            ArmInstruction::HalfwordTransfer => {
                "Transferencia de media palabra/byte con signo (LDRH / STRH / ...)"
            }
            ArmInstruction::SingleDataTransfer => "Transferencia de datos (LDR / STR)",
            ArmInstruction::BlockDataTransfer => "Transferencia en bloque (LDM / STM)",
            ArmInstruction::Coprocessor => "Operación de coprocesador (sin uso en GBA)",
            ArmInstruction::SoftwareInterrupt => "Interrupción software (SWI / llamada a BIOS)",
            ArmInstruction::Undefined => "Instrucción indefinida",
        };
        f.write_str(s)
    }
}

/// Resultado del decode ARM en dos pasos.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decoded {
    /// La condición (bits 31-28) **no** se cumple con el CPSR actual: la
    /// instrucción se descarta y actúa como un NOP de un ciclo. Se conserva la
    /// condición evaluada por si es útil al depurar.
    ConditionFailed(Condition),
    /// La condición se cumple; la instrucción identificada es esta.
    Execute(ArmInstruction),
}

/// **Decode ARM en dos pasos** (Mini-Hito 2.1c): primero la condición contra el
/// `cpsr`, y solo si se cumple, la clasificación del opcode.
///
/// Ver la cabecera del módulo para el porqué de separar ambos pasos.
pub fn decode(instr: u32, cpsr: Cpsr) -> Decoded {
    let cond = Condition::from_instr(instr);
    if cond.passes(cpsr) {
        Decoded::Execute(ArmInstruction::decode(instr))
    } else {
        Decoded::ConditionFailed(cond)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrae_la_condicion_de_los_bits_31_28() {
        // 0xEA00002E → nibble alto 0xE = AL (Always).
        assert_eq!(Condition::from_instr(0xEA00_002E), Condition::Al);
        // 0x0A00002E → nibble alto 0x0 = EQ.
        assert_eq!(Condition::from_instr(0x0A00_002E), Condition::Eq);
        // 0x1A00002E → 0x1 = NE.
        assert_eq!(Condition::from_instr(0x1A00_002E), Condition::Ne);
    }

    #[test]
    fn al_siempre_pasa_y_nv_nunca() {
        let cpsr = Cpsr::from_bits(0);
        assert!(Condition::Al.passes(cpsr));
        assert!(!Condition::Nv.passes(cpsr));
    }

    #[test]
    fn eq_y_ne_dependen_del_flag_z() {
        let mut cpsr = Cpsr::from_bits(0);
        cpsr.set_z(true);
        assert!(Condition::Eq.passes(cpsr));
        assert!(!Condition::Ne.passes(cpsr));
        cpsr.set_z(false);
        assert!(!Condition::Eq.passes(cpsr));
        assert!(Condition::Ne.passes(cpsr));
    }

    #[test]
    fn condiciones_con_signo_ge_y_lt() {
        let mut cpsr = Cpsr::from_bits(0);
        // N == V → GE pasa, LT no.
        cpsr.set_n(true);
        cpsr.set_v(true);
        assert!(Condition::Ge.passes(cpsr));
        assert!(!Condition::Lt.passes(cpsr));
        // N != V → al revés.
        cpsr.set_v(false);
        assert!(!Condition::Ge.passes(cpsr));
        assert!(Condition::Lt.passes(cpsr));
    }

    #[test]
    fn clasifica_el_salto_b_y_bl() {
        // El ejemplo del plan: 0xEA00002E es un B.
        assert_eq!(
            ArmInstruction::decode(0xEA00_002E),
            ArmInstruction::Branch { link: false }
        );
        // 0xEB... tiene el bit 24 (L) a 1 → BL.
        assert_eq!(
            ArmInstruction::decode(0xEB00_002E),
            ArmInstruction::Branch { link: true }
        );
    }

    #[test]
    fn clasifica_los_formatos_principales() {
        // MOV r0, #5
        assert_eq!(ArmInstruction::decode(0xE3A0_0005), ArmInstruction::DataProcessing);
        // MRS r0, CPSR
        assert_eq!(ArmInstruction::decode(0xE10F_0000), ArmInstruction::PsrTransfer);
        // MUL r0, r1, r0
        assert_eq!(ArmInstruction::decode(0xE000_0091), ArmInstruction::Multiply);
        // BX lr
        assert_eq!(ArmInstruction::decode(0xE12F_FF1E), ArmInstruction::BranchExchange);
        // LDR r1, [r0]
        assert_eq!(
            ArmInstruction::decode(0xE590_1000),
            ArmInstruction::SingleDataTransfer
        );
        // LDM sp!, {pc}
        assert_eq!(
            ArmInstruction::decode(0xE8BD_8000),
            ArmInstruction::BlockDataTransfer
        );
        // SWI #0
        assert_eq!(
            ArmInstruction::decode(0xEF00_0000),
            ArmInstruction::SoftwareInterrupt
        );
    }

    #[test]
    fn decode_dos_pasos_descarta_si_la_condicion_falla() {
        // 0x0A00002E es "BEQ": salta solo si Z = 1.
        let mut cpsr = Cpsr::from_bits(0); // Z = 0

        // Z = 0 → la condición EQ no se cumple → se descarta (NOP).
        assert_eq!(decode(0x0A00_002E, cpsr), Decoded::ConditionFailed(Condition::Eq));

        // Z = 1 → ahora sí se ejecuta y se identifica como salto.
        cpsr.set_z(true);
        assert_eq!(
            decode(0x0A00_002E, cpsr),
            Decoded::Execute(ArmInstruction::Branch { link: false })
        );
    }

    #[test]
    fn el_texto_del_salto_coincide_con_la_prueba_del_plan() {
        let kind = ArmInstruction::decode(0xEA00_002E);
        assert_eq!(format!("{kind}"), "Salto (B / Branch)");
    }
}
