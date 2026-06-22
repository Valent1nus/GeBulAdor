//! Decodificación de instrucciones en **modo THUMB** (16 bits).
//!
//! Mini-Hito 2.1c-bis. Identifica *qué formato* es una palabra THUMB de 16 bits,
//! **sin ejecutar su lógica** todavía.
//!
//! ## ⚠️ THUMB no es "ARM comprimido"
//!
//! Es el error conceptual que el plan quiere evitar. THUMB es un *set de
//! instrucciones propio* de 16 bits, no un subconjunto recortado de ARM:
//!
//! - **No hay código de condición embebido.** En ARM, los bits 31-28 de *toda*
//!   instrucción son una condición ([`crate::arm::Condition`]); en THUMB no
//!   existe tal campo. La única instrucción condicional es el salto `B<cond>`
//!   (formato 16), donde la condición va dentro de su propio opcode. Por eso
//!   este decoder **no** hace el flujo de dos pasos de ARM: clasifica directo.
//! - **Menos bits para inmediatos** y **acceso limitado a `r8`–`r15`** en la
//!   mayoría de instrucciones de registro general.
//! - **Su propia tabla de 19 formatos**, distinguidos por los bits altos
//!   (15-13) y refinados con bits inferiores.
//!
//! Tratar THUMB como "un caso particular de ARM" produce un `match` con ramas
//! mal mapeadas que parecen funcionar con instrucciones simples y fallan con las
//! reales. Por eso este módulo y [`ThumbInstruction::decode`] son **totalmente
//! separados** de los de [`crate::arm`].

use std::fmt;

/// Formato de una instrucción THUMB, según la tabla de 19 formatos del
/// ARM7TDMI. De momento solo se **clasifica**: ninguna variante lleva todavía
/// los campos descodificados ni lógica de ejecución (eso es el Mini-Hito 2.1d).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThumbInstruction {
    /// Formato 1: desplazamiento de registro (`LSL`/`LSR`/`ASR` por inmediato).
    MoveShifted,
    /// Formato 2: suma/resta de registro o inmediato de 3 bits (`ADD`/`SUB`).
    AddSubtract,
    /// Formato 3: `MOV`/`CMP`/`ADD`/`SUB` con inmediato de 8 bits.
    MoveCompareAddSubImm,
    /// Formato 4: operaciones ALU registro-registro (`AND`, `EOR`, `LSL`...).
    AluOperation,
    /// Formato 5: operaciones con registros altos (`r8`–`r15`) y `BX`.
    HiRegisterOpBx,
    /// Formato 6: carga relativa al `PC` (`LDR Rd, [PC, #imm]`).
    PcRelativeLoad,
    /// Formato 7: carga/almacén con offset de registro (`LDR`/`STR`/`LDRB`/`STRB`).
    LoadStoreRegOffset,
    /// Formato 8: carga/almacén de byte/media palabra con signo.
    LoadStoreSignExtended,
    /// Formato 9: carga/almacén con offset inmediato.
    LoadStoreImmOffset,
    /// Formato 10: carga/almacén de media palabra (`LDRH`/`STRH`).
    LoadStoreHalfword,
    /// Formato 11: carga/almacén relativo al `SP`.
    SpRelativeLoadStore,
    /// Formato 12: cálculo de dirección (`ADD Rd, PC/SP, #imm`).
    LoadAddress,
    /// Formato 13: ajuste del `SP` por un inmediato con signo (`ADD SP, #imm`).
    AddOffsetToSp,
    /// Formato 14: `PUSH`/`POP` de registros.
    PushPop,
    /// Formato 15: carga/almacén múltiple (`LDMIA`/`STMIA`).
    MultipleLoadStore,
    /// Formato 16: salto condicional (`B<cond>`), la única condicional de THUMB.
    ConditionalBranch,
    /// Formato 17: interrupción software (`SWI`, llamadas a la BIOS).
    SoftwareInterrupt,
    /// Formato 18: salto incondicional (`B`).
    UnconditionalBranch,
    /// Formato 19: salto largo con enlace (`BL`), codificado en dos halfwords.
    LongBranchWithLink,
    /// Patrón de bits que no corresponde a ninguna instrucción THUMB de ARMv4T.
    Undefined,
}

impl ThumbInstruction {
    /// Clasifica una instrucción THUMB de 16 bits.
    ///
    /// Es la "tabla de formatos": se decide por los 3 bits altos (15-13) y se
    /// refina con bits inferiores donde varios formatos comparten prefijo. No
    /// hay paso de condición previo (THUMB no la lleva embebida).
    pub fn decode(instr: u16) -> ThumbInstruction {
        use ThumbInstruction::*;
        match instr >> 13 {
            // 000: desplazamiento (F1) o add/sub (F2, prefijo 00011).
            0b000 => {
                if (instr >> 11) & 0b11 == 0b11 {
                    AddSubtract
                } else {
                    MoveShifted
                }
            }
            // 001: mov/cmp/add/sub con inmediato de 8 bits (F3).
            0b001 => MoveCompareAddSubImm,
            // 010: varios formatos según los bits 12-9.
            0b010 => {
                if (instr >> 12) & 1 == 1 {
                    // 0101 xx_: offset de registro (F7) o con signo (F8).
                    if (instr >> 9) & 1 == 1 {
                        LoadStoreSignExtended
                    } else {
                        LoadStoreRegOffset
                    }
                } else if (instr >> 11) & 1 == 1 {
                    PcRelativeLoad // 01001 (F6)
                } else if (instr >> 10) & 1 == 1 {
                    HiRegisterOpBx // 010001 (F5)
                } else {
                    AluOperation // 010000 (F4)
                }
            }
            // 011: carga/almacén con offset inmediato (F9).
            0b011 => LoadStoreImmOffset,
            // 100: media palabra (F10, 1000) o relativo al SP (F11, 1001).
            0b100 => {
                if (instr >> 12) & 1 == 1 {
                    SpRelativeLoadStore
                } else {
                    LoadStoreHalfword
                }
            }
            // 101: load address (F12), ajuste de SP (F13) o push/pop (F14).
            0b101 => {
                if (instr >> 12) & 1 == 0 {
                    LoadAddress // 1010
                } else if (instr >> 8) & 0b1111 == 0b0000 {
                    AddOffsetToSp // 1011 0000
                } else if (instr >> 9) & 0b11 == 0b10 {
                    PushPop // 1011 x10
                } else {
                    Undefined
                }
            }
            // 110: load/store múltiple (F15, 1100) o salto cond./SWI (1101).
            0b110 => {
                if (instr >> 12) & 1 == 0 {
                    MultipleLoadStore
                } else {
                    // 1101 cccc: cond=1111 → SWI (F17), cond=1110 → indefinido,
                    // resto → B<cond> (F16).
                    match (instr >> 8) & 0b1111 {
                        0b1111 => SoftwareInterrupt,
                        0b1110 => Undefined,
                        _ => ConditionalBranch,
                    }
                }
            }
            // 111: salto incondicional (F18, 11100) o BL largo (F19, 1111x).
            0b111 => match (instr >> 11) & 0b11 {
                0b00 => UnconditionalBranch,
                0b10 | 0b11 => LongBranchWithLink,
                // 11101 es BLX en ARMv5; no existe en el ARM7TDMI (ARMv4T).
                _ => Undefined,
            },
            _ => unreachable!("instr >> 13 es de 3 bits (0..=7)"),
        }
    }
}

impl fmt::Display for ThumbInstruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            ThumbInstruction::MoveShifted => "Desplazamiento de registro (formato 1 THUMB: LSL/LSR/ASR)",
            ThumbInstruction::AddSubtract => "Suma/resta (formato 2 THUMB: ADD/SUB)",
            ThumbInstruction::MoveCompareAddSubImm => "MOV/CMP/ADD/SUB con inmediato (formato 3 THUMB)",
            ThumbInstruction::AluOperation => "Operación ALU (formato 4 THUMB)",
            ThumbInstruction::HiRegisterOpBx => "Registros altos / BX (formato 5 THUMB)",
            ThumbInstruction::PcRelativeLoad => "Carga relativa al PC (formato 6 THUMB)",
            ThumbInstruction::LoadStoreRegOffset => "Carga/almacén con offset de registro (formato 7 THUMB)",
            ThumbInstruction::LoadStoreSignExtended => "Carga/almacén con signo (formato 8 THUMB)",
            ThumbInstruction::LoadStoreImmOffset => "Carga/almacén con offset inmediato (formato 9 THUMB)",
            ThumbInstruction::LoadStoreHalfword => "Carga/almacén de media palabra (formato 10 THUMB)",
            ThumbInstruction::SpRelativeLoadStore => "Carga/almacén relativo al SP (formato 11 THUMB)",
            ThumbInstruction::LoadAddress => "Cálculo de dirección (formato 12 THUMB)",
            ThumbInstruction::AddOffsetToSp => "Ajuste del SP (formato 13 THUMB)",
            ThumbInstruction::PushPop => "PUSH/POP de registros (formato 14 THUMB)",
            ThumbInstruction::MultipleLoadStore => "Carga/almacén múltiple (formato 15 THUMB)",
            ThumbInstruction::ConditionalBranch => "Salto condicional (formato 16 THUMB: B<cond>)",
            ThumbInstruction::SoftwareInterrupt => "Interrupción software (formato 17 THUMB: SWI)",
            ThumbInstruction::UnconditionalBranch => "Salto incondicional (formato 18 THUMB: B)",
            ThumbInstruction::LongBranchWithLink => "Salto largo con enlace (formato 19 THUMB: BL)",
            ThumbInstruction::Undefined => "Instrucción THUMB indefinida",
        };
        f.write_str(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn la_prueba_del_plan_mov_r0_5() {
        // 0x2005 = «MOV r0, #5» en THUMB → formato 3, distinto a cualquier
        // formato de ARM (que ni siquiera comparte el ancho de 16 bits).
        assert_eq!(ThumbInstruction::decode(0x2005), ThumbInstruction::MoveCompareAddSubImm);
        assert_eq!(
            format!("{}", ThumbInstruction::decode(0x2005)),
            "MOV/CMP/ADD/SUB con inmediato (formato 3 THUMB)"
        );
    }

    #[test]
    fn cubre_los_formatos_principales() {
        let casos = [
            (0x0000u16, ThumbInstruction::MoveShifted),          // LSL r0,r0,#0
            (0x1C00, ThumbInstruction::AddSubtract),             // ADD r0,r0,#0
            (0x2005, ThumbInstruction::MoveCompareAddSubImm),    // MOV r0,#5
            (0x4000, ThumbInstruction::AluOperation),            // AND r0,r0
            (0x4700, ThumbInstruction::HiRegisterOpBx),          // BX r0
            (0x4800, ThumbInstruction::PcRelativeLoad),          // LDR r0,[PC,#0]
            (0x5000, ThumbInstruction::LoadStoreRegOffset),      // STR r0,[r0,r0]
            (0x5200, ThumbInstruction::LoadStoreSignExtended),   // STRH r0,[r0,r0]
            (0x6000, ThumbInstruction::LoadStoreImmOffset),      // STR r0,[r0,#0]
            (0x8000, ThumbInstruction::LoadStoreHalfword),       // STRH r0,[r0,#0]
            (0x9000, ThumbInstruction::SpRelativeLoadStore),     // STR r0,[SP,#0]
            (0xA000, ThumbInstruction::LoadAddress),             // ADD r0,PC,#0
            (0xB000, ThumbInstruction::AddOffsetToSp),           // ADD SP,#0
            (0xB400, ThumbInstruction::PushPop),                 // PUSH {}
            (0xC000, ThumbInstruction::MultipleLoadStore),       // STMIA r0!,{}
            (0xD000, ThumbInstruction::ConditionalBranch),       // BEQ
            (0xDF00, ThumbInstruction::SoftwareInterrupt),       // SWI #0
            (0xE000, ThumbInstruction::UnconditionalBranch),     // B
            (0xF000, ThumbInstruction::LongBranchWithLink),      // BL (1ª mitad)
        ];
        for (instr, esperado) in casos {
            assert_eq!(
                ThumbInstruction::decode(instr),
                esperado,
                "fallo decodificando {instr:#06X}"
            );
        }
    }

    #[test]
    fn los_huecos_de_arm7tdmi_son_indefinidos() {
        // 0xE800 = 11101... : sería BLX en ARMv5, indefinido en el ARM7TDMI.
        assert_eq!(ThumbInstruction::decode(0xE800), ThumbInstruction::Undefined);
        // 0xDE00 = 1101 1110 : cond=1110, hueco reservado del salto condicional.
        assert_eq!(ThumbInstruction::decode(0xDE00), ThumbInstruction::Undefined);
    }
}
