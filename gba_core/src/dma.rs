//! El **DMA** (Direct Memory Access) de la GBA: cuatro canales que copian bloques
//! de memoria **sin** intervención de la CPU (Mini-Hito 2.3b).
//!
//! ## Qué hace el DMA y por qué existe
//!
//! Mover muchos datos con la CPU (un bucle de `LDR`/`STR`) es lento y la mantiene
//! ocupada. El DMA es hardware dedicado a eso: le das una dirección de **origen**,
//! una de **destino** y una **cantidad**, y él copia el bloque mientras la CPU se
//! detiene (le "roba" el bus). La GBA tiene **cuatro canales** (DMA0–DMA3) con
//! prioridad fija (0 = la más alta) y usos preferentes: DMA0 para cosas críticas,
//! DMA1/DMA2 para alimentar el sonido (FIFO) y DMA3 de propósito general (cargar
//! gráficos, copiar grandes bloques).
//!
//! ## Alcance de este hito: solo la **copia inmediata**
//!
//! Un canal arranca según su **modo de disparo** (bits 12-13 del control):
//!
//! | Modo | Cuándo arranca | Estado |
//! |---|---|---|
//! | 0 Inmediato | en cuanto se activa el `enable` | **implementado aquí (2.3b)** |
//! | 1 V-Blank   | al entrar en V-Blank | pendiente de la PPU (2.4b) |
//! | 2 H-Blank   | al entrar en H-Blank | pendiente de la PPU (2.4b) |
//! | 3 Especial  | FIFO de sonido / captura | pendiente de la APU (2.5b) |
//!
//! Este módulo implementa el **modo inmediato**: la copia ocurre al escribir el
//! registro de control con el `enable` a 1. Los demás modos quedan **armados**
//! (el `enable` se queda a 1) pero no disparan nada todavía: lo harán cuando
//! existan sus eventos (PPU/APU), que reusarán el [`crate::Scheduler`].
//!
//! ## Dónde vive la lógica (reparto con el [`crate::Bus`])
//!
//! Este módulo es la **fuente de verdad** de los registros DMA y decide *qué*
//! copiar (origen, destino, cantidad, paso), pero **no** toca memoria: la copia en
//! sí la hace el [`crate::Bus`] (es quien tiene `read_*`/`write_*`). El bus
//! enruta a este módulo las lecturas/escrituras del rango de registros DMA y, tras
//! una escritura al control, le pregunta si hay que disparar una transferencia
//! ([`Dma::poll_channel`]) y le pide el "plan" de copia ([`Dma::plan`]).
//!
//! ## 🛡️ Seguridad — origen/destino/cantidad los controla la ROM
//!
//! El juego escribe libremente las direcciones y la cantidad. Dos defensas:
//! 1. **Toda lectura/escritura de la copia pasa por el `Bus`**, que ya hace
//!    *clamp* y nunca panica ante una dirección fuera de mapa.
//! 2. **La cantidad está acotada por el propio hardware**: el campo de conteo es
//!    de 14 bits (DMA0–2, máx. `0x4000` unidades) o 16 bits (DMA3, máx. `0x1_0000`),
//!    así que un canal nunca puede pedir un bucle de copia ilimitado.

/// Número de canales de DMA del hardware (DMA0–DMA3).
pub const DMA_CHANNELS: usize = 4;

/// Offset (dentro de la región de I/O, base `0x0400_0000`) del primer registro
/// DMA (`DMA0SAD`). Los registros ocupan de `0x0B0` a `0x0E0` (exclusivo).
const DMA_IO_BASE: u32 = 0x0B0;
/// Fin (exclusivo) del bloque de registros DMA dentro de la región de I/O.
const DMA_IO_END: u32 = 0x0E0;
/// Bytes que ocupa cada canal: `SAD`(4) + `DAD`(4) + `CNT_L`(2) + `CNT_H`(2) = 12.
const CHANNEL_STRIDE: usize = 0x0C;
/// Bytes totales de todos los registros DMA: 4 canales × 12 = 48.
const DMA_REGS_LEN: usize = DMA_CHANNELS * CHANNEL_STRIDE;

// Offsets de cada registro **dentro** de su canal.
/// `DMAxSAD` (Source Address, 32 bits): dirección de origen.
const OFF_SAD: usize = 0x0;
/// `DMAxDAD` (Destination Address, 32 bits): dirección de destino.
const OFF_DAD: usize = 0x4;
/// `DMAxCNT_L` (Word Count, 16 bits): número de unidades a transferir.
const OFF_CNT_L: usize = 0x8;
/// `DMAxCNT_H` (Control, 16 bits): modo de la transferencia y bit de arranque.
const OFF_CNT_H: usize = 0xA;

// Bits del registro de control `DMAxCNT_H`.
/// Control de la dirección de **destino** (bits 5-6): 0=incrementa, 1=decrementa,
/// 2=fija, 3=incrementa con recarga.
const CTRL_DST_CTL_SHIFT: u16 = 5;
/// Control de la dirección de **origen** (bits 7-8): 0=incrementa, 1=decrementa,
/// 2=fija, 3=prohibido.
const CTRL_SRC_CTL_SHIFT: u16 = 7;
/// Tipo de transferencia (bit 10): 0 = 16 bits, 1 = 32 bits.
const CTRL_WORD_BIT: u16 = 1 << 10;
/// Modo de arranque (bits 12-13): 0=inmediato, 1=V-Blank, 2=H-Blank, 3=especial.
const CTRL_TIMING_SHIFT: u16 = 12;
/// Bit de **enable** (bit 15): activa el canal. Su flanco 0→1 es lo que dispara.
const CTRL_ENABLE_BIT: u16 = 1 << 15;

/// El modo de arranque "inmediato" (campo de timing = 0): único implementado en
/// el Mini-Hito 2.3b.
const TIMING_IMMEDIATE: u16 = 0;

/// Máscara de la dirección de **origen** por canal. DMA0 solo direcciona memoria
/// interna (27 bits); DMA1–3 alcanzan también el cartucho (28 bits).
const SRC_ADDR_MASK: [u32; DMA_CHANNELS] = [0x07FF_FFFF, 0x0FFF_FFFF, 0x0FFF_FFFF, 0x0FFF_FFFF];
/// Máscara de la dirección de **destino** por canal. Solo DMA3 puede escribir al
/// espacio del cartucho (28 bits); DMA0–2 se quedan en memoria interna (27 bits).
const DST_ADDR_MASK: [u32; DMA_CHANNELS] = [0x07FF_FFFF, 0x07FF_FFFF, 0x07FF_FFFF, 0x0FFF_FFFF];
/// Máscara del contador por canal: 14 bits en DMA0–2, 16 bits en DMA3. Un conteo
/// de 0 significa "el máximo" (máscara + 1), no "nada" (ver [`Dma::plan`]).
const COUNT_MASK: [u32; DMA_CHANNELS] = [0x3FFF, 0x3FFF, 0x3FFF, 0xFFFF];

/// El **plan** de una transferencia ya decodificado del control de un canal: lo
/// produce [`Dma::plan`] y lo ejecuta el [`crate::Bus`]. Separa el "qué copiar"
/// (aquí, sin tocar memoria) del "copiar" (el bus, que sí accede a la RAM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaTransfer {
    /// Dirección de origen, ya enmascarada al canal y alineada a la unidad.
    pub src: u32,
    /// Dirección de destino, ya enmascarada al canal y alineada a la unidad.
    pub dst: u32,
    /// Número de unidades a transferir (nunca 0: un 0 ya se tradujo al máximo).
    pub count: u32,
    /// `true` si cada unidad es de 32 bits; `false` si es de 16 bits.
    pub word: bool,
    /// Cuánto avanza `src` tras cada unidad (con signo: negativo si decrementa).
    pub src_step: i32,
    /// Cuánto avanza `dst` tras cada unidad (con signo: negativo si decrementa).
    pub dst_step: i32,
}

/// El controlador de DMA: los registros de los cuatro canales y el estado mínimo
/// para detectar el flanco de arranque.
///
/// Vive **dentro** del [`crate::Bus`] (que enruta aquí los accesos al rango de
/// registros DMA). Es la fuente de verdad de `SAD`/`DAD`/`CNT_L`/`CNT_H`.
pub struct Dma {
    /// Bytes crudos de los 48 registros (4 canales × 12 bytes), en el mismo orden
    /// y *little-endian* que la región de I/O. Es la fuente de verdad interna; las
    /// lecturas externas se filtran en [`Dma::read_u8`] (los registros write-only
    /// devuelven 0).
    regs: [u8; DMA_REGS_LEN],

    /// Último valor visto del bit `enable` de cada canal, para detectar el flanco
    /// 0→1 que dispara la transferencia (y no re-disparar si ya estaba activo).
    enabled_latch: [bool; DMA_CHANNELS],

    /// `true` mientras se ejecuta una transferencia. Es una **guarda de
    /// reentrada**: si la copia escribiera por casualidad sobre un registro DMA,
    /// evita que se dispare otra transferencia anidada (el bus consulta este flag
    /// antes de sondear disparos). Ver [`Dma::is_running`].
    running: bool,
}

impl Dma {
    /// Crea un controlador de DMA en reposo: todos los registros a cero, ningún
    /// canal activo.
    pub fn new() -> Self {
        Dma {
            regs: [0; DMA_REGS_LEN],
            enabled_latch: [false; DMA_CHANNELS],
            running: false,
        }
    }

    /// `true` si el offset de I/O `io_off` (los 24 bits bajos de la dirección) cae
    /// dentro del bloque de registros DMA. Lo usa el bus para enrutar aquí las
    /// lecturas/escrituras de byte.
    pub fn in_range(io_off: u32) -> bool {
        (DMA_IO_BASE..DMA_IO_END).contains(&io_off)
    }

    /// `true` si una escritura de `width` bytes a partir del offset de I/O `io_off`
    /// **solapa** el bloque de registros DMA. El bus lo consulta tras una escritura
    /// de 16/32 bits para decidir si debe sondear un posible disparo.
    pub fn touches(io_off: u32, width: u32) -> bool {
        io_off < DMA_IO_END && io_off + width > DMA_IO_BASE
    }

    /// Lee un byte de un registro DMA. Respeta que `SAD`/`DAD`/`CNT_L` son
    /// **write-only** en el hardware (su lectura devuelve 0); solo el control
    /// `CNT_H` es legible. Nunca panica ante un offset inesperado.
    pub fn read_u8(&self, io_off: u32) -> u8 {
        let local = io_off.wrapping_sub(DMA_IO_BASE) as usize;
        // Dentro de cada canal, solo los bytes del control (offsets 10–11) son
        // legibles; el resto (origen/destino/contador) son write-only.
        if local % CHANNEL_STRIDE >= OFF_CNT_H {
            self.regs.get(local).copied().unwrap_or(0)
        } else {
            0
        }
    }

    /// Escribe un byte en un registro DMA (fuente de verdad interna). **No** dispara
    /// la transferencia: el disparo lo decide el bus tras la escritura de 16/32
    /// bits, vía [`Dma::poll_channel`]. Nunca panica ante un offset inesperado.
    pub fn write_u8(&mut self, io_off: u32, value: u8) {
        let local = io_off.wrapping_sub(DMA_IO_BASE) as usize;
        if let Some(slot) = self.regs.get_mut(local) {
            *slot = value;
        }
    }

    /// `true` mientras una transferencia está en curso (guarda de reentrada).
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Marca el inicio de una transferencia (activa la guarda de reentrada).
    pub fn begin(&mut self) {
        self.running = true;
    }

    /// Marca el fin de una transferencia (libera la guarda de reentrada).
    pub fn end(&mut self) {
        self.running = false;
    }

    /// Actualiza el latch de `enable` del canal `ch` con el valor actual de su
    /// control y devuelve `true` si **ahora mismo** hay que ejecutar una
    /// transferencia **inmediata**.
    ///
    /// Solo dispara en el **flanco** 0→1 del `enable` y solo si el modo de arranque
    /// es inmediato (timing 0). Si el modo es V-Blank/H-Blank/especial, el canal
    /// queda **armado** (latch a 1, no se re-dispara) a la espera de su evento, que
    /// llegará con la PPU (2.4b) o la APU (2.5b).
    pub fn poll_channel(&mut self, ch: usize) -> bool {
        let control = self.control(ch);
        let enable = control & CTRL_ENABLE_BIT != 0;
        let was_enabled = self.enabled_latch.get(ch).copied().unwrap_or(false);
        if let Some(latch) = self.enabled_latch.get_mut(ch) {
            *latch = enable;
        }

        if enable && !was_enabled {
            let timing = (control >> CTRL_TIMING_SHIFT) & 0b11;
            return timing == TIMING_IMMEDIATE;
        }
        false
    }

    /// Decodifica el control del canal `ch` en un [`DmaTransfer`] listo para que el
    /// bus lo ejecute: direcciones enmascaradas y alineadas, conteo resuelto (0 →
    /// máximo) y el paso con signo de origen/destino según su modo de dirección.
    pub fn plan(&self, ch: usize) -> DmaTransfer {
        let control = self.control(ch);
        let word = control & CTRL_WORD_BIT != 0;
        let unit: u32 = if word { 4 } else { 2 };

        // Direcciones: enmascaradas al rango del canal y alineadas a la unidad
        // (una transferencia de 32 bits ignora los 2 bits bajos; la de 16, el bajo).
        let src = self.reg_u32(ch, OFF_SAD) & SRC_ADDR_MASK[ch] & !(unit - 1);
        let dst = self.reg_u32(ch, OFF_DAD) & DST_ADDR_MASK[ch] & !(unit - 1);

        // Conteo: enmascarado al ancho del canal; un 0 significa el máximo.
        let raw_count = self.reg_u16(ch, OFF_CNT_L) as u32 & COUNT_MASK[ch];
        let count = if raw_count == 0 { COUNT_MASK[ch] + 1 } else { raw_count };

        let src_ctl = (control >> CTRL_SRC_CTL_SHIFT) & 0b11;
        let dst_ctl = (control >> CTRL_DST_CTL_SHIFT) & 0b11;
        DmaTransfer {
            src,
            dst,
            count,
            word,
            src_step: addr_step(src_ctl, unit, false),
            dst_step: addr_step(dst_ctl, unit, true),
        }
    }

    /// Cierra una transferencia **inmediata**: como el modo inmediato es de disparo
    /// único (el bit `repeat` no aplica), limpia el `enable` (bit 15) del control y
    /// el latch del canal, de modo que una lectura posterior de `CNT_H` vea el
    /// canal ya parado y un nuevo `enable` vuelva a ser un flanco que dispare.
    pub fn finish_immediate(&mut self, ch: usize) {
        // El bit 15 (enable) es el bit 7 del byte alto del control `CNT_H`.
        let hi_byte = ch * CHANNEL_STRIDE + OFF_CNT_H + 1;
        if let Some(slot) = self.regs.get_mut(hi_byte) {
            *slot &= !0x80;
        }
        if let Some(latch) = self.enabled_latch.get_mut(ch) {
            *latch = false;
        }
    }

    /// El registro de control `CNT_H` (16 bits) del canal `ch`.
    fn control(&self, ch: usize) -> u16 {
        self.reg_u16(ch, OFF_CNT_H)
    }

    /// Lee 16 bits *little-endian* del registro en `off` (offset dentro del canal)
    /// del canal `ch`. Nunca panica.
    fn reg_u16(&self, ch: usize, off: usize) -> u16 {
        let i = ch * CHANNEL_STRIDE + off;
        let lo = self.regs.get(i).copied().unwrap_or(0) as u16;
        let hi = self.regs.get(i + 1).copied().unwrap_or(0) as u16;
        lo | (hi << 8)
    }

    /// Lee 32 bits *little-endian* del registro en `off` (offset dentro del canal)
    /// del canal `ch`. Nunca panica.
    fn reg_u32(&self, ch: usize, off: usize) -> u32 {
        let i = ch * CHANNEL_STRIDE + off;
        let byte = |k: usize| self.regs.get(i + k).copied().unwrap_or(0) as u32;
        byte(0) | (byte(1) << 8) | (byte(2) << 16) | (byte(3) << 24)
    }
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

/// Paso (con signo) que aplica un modo de control de dirección sobre una dirección
/// tras cada unidad transferida. `unit` es 2 o 4 bytes; `is_dst` distingue el modo
/// 3, que en destino es "incrementa con recarga" y en origen está prohibido.
fn addr_step(ctl: u16, unit: u32, is_dst: bool) -> i32 {
    let unit = unit as i32;
    match ctl {
        0 => unit,  // incrementa
        1 => -unit, // decrementa
        2 => 0,     // fija
        // 3: en destino, "incrementa con recarga" (la recarga solo afecta al modo
        // repeat, ajeno al inmediato → se comporta como incremento); en origen es
        // un valor prohibido → lo tratamos como dirección fija (defensa).
        _ => {
            if is_dst {
                unit
            } else {
                0
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Offset de I/O del registro `off` (dentro del canal) del canal `ch`.
    fn off_of(ch: usize, off: usize) -> u32 {
        DMA_IO_BASE + (ch * CHANNEL_STRIDE + off) as u32
    }

    /// Escribe un valor de 16 bits en un registro (dos `write_u8` little-endian).
    fn write_reg16(dma: &mut Dma, ch: usize, off: usize, value: u16) {
        let base = off_of(ch, off);
        dma.write_u8(base, value as u8);
        dma.write_u8(base + 1, (value >> 8) as u8);
    }

    /// Escribe un valor de 32 bits en un registro (cuatro `write_u8` LE).
    fn write_reg32(dma: &mut Dma, ch: usize, off: usize, value: u32) {
        let base = off_of(ch, off);
        for k in 0..4 {
            dma.write_u8(base + k, (value >> (8 * k)) as u8);
        }
    }

    #[test]
    fn el_rango_de_registros_dma_es_0xb0_a_0xe0() {
        assert!(!Dma::in_range(0x0AF));
        assert!(Dma::in_range(0x0B0)); // DMA0SAD
        assert!(Dma::in_range(0x0DF)); // último byte de DMA3CNT_H
        assert!(!Dma::in_range(0x0E0));
    }

    #[test]
    fn touches_detecta_solapamiento_de_una_escritura() {
        // Un word en DMA0CNT_L (0xB8) cubre 0xB8..0xBC, que incluye CNT_H (0xBA).
        assert!(Dma::touches(0x0B8, 4));
        // Un halfword justo antes del rango no lo toca.
        assert!(!Dma::touches(0x0AE, 2));
        // Pero un word a 0x0AE (0xAE..0xB2) sí entra en el rango.
        assert!(Dma::touches(0x0AE, 4));
    }

    #[test]
    fn origen_destino_y_contador_son_write_only_al_leer() {
        let mut dma = Dma::new();
        write_reg32(&mut dma, 0, OFF_SAD, 0x0203_0405);
        write_reg32(&mut dma, 0, OFF_DAD, 0x0607_0809);
        write_reg16(&mut dma, 0, OFF_CNT_L, 0x1234);
        // Leerlos devuelve 0 (write-only), aunque internamente sí se guardaron.
        assert_eq!(dma.read_u8(off_of(0, OFF_SAD)), 0);
        assert_eq!(dma.read_u8(off_of(0, OFF_DAD)), 0);
        assert_eq!(dma.read_u8(off_of(0, OFF_CNT_L)), 0);
        // El control sí es legible.
        write_reg16(&mut dma, 0, OFF_CNT_H, 0xABCD);
        assert_eq!(dma.read_u8(off_of(0, OFF_CNT_H)), 0xCD);
        assert_eq!(dma.read_u8(off_of(0, OFF_CNT_H) + 1), 0xAB);
    }

    #[test]
    fn poll_dispara_solo_en_el_flanco_de_enable_inmediato() {
        let mut dma = Dma::new();
        // Control: enable (bit 15) + timing inmediato (0).
        write_reg16(&mut dma, 0, OFF_CNT_H, CTRL_ENABLE_BIT);
        // Primer sondeo: flanco 0→1 → dispara.
        assert!(dma.poll_channel(0));
        // Segundo sondeo sin cambios: ya no es flanco → no re-dispara.
        assert!(!dma.poll_channel(0));
    }

    #[test]
    fn un_canal_no_inmediato_queda_armado_sin_disparar() {
        let mut dma = Dma::new();
        // Control: enable + timing 1 (V-Blank): no se dispara en modo inmediato.
        let vblank = CTRL_ENABLE_BIT | (1 << CTRL_TIMING_SHIFT);
        write_reg16(&mut dma, 0, OFF_CNT_H, vblank);
        assert!(!dma.poll_channel(0), "V-Blank no dispara como inmediato");
        // Y queda armado (latch a 1): un nuevo sondeo tampoco lo re-dispara.
        assert!(!dma.poll_channel(0));
    }

    #[test]
    fn plan_decodifica_una_copia_de_32_bits_incremental() {
        let mut dma = Dma::new();
        write_reg32(&mut dma, 3, OFF_SAD, 0x0200_0000);
        write_reg32(&mut dma, 3, OFF_DAD, 0x0300_0000);
        write_reg16(&mut dma, 3, OFF_CNT_L, 4);
        // 32 bits (bit 10) + enable; origen/destino incrementan (modo 0).
        write_reg16(&mut dma, 3, OFF_CNT_H, CTRL_WORD_BIT | CTRL_ENABLE_BIT);

        let plan = dma.plan(3);
        assert_eq!(
            plan,
            DmaTransfer {
                src: 0x0200_0000,
                dst: 0x0300_0000,
                count: 4,
                word: true,
                src_step: 4,
                dst_step: 4,
            }
        );
    }

    #[test]
    fn plan_resuelve_conteo_cero_como_el_maximo_del_canal() {
        let mut dma = Dma::new();
        // DMA0 (14 bits): conteo 0 → 0x4000.
        write_reg16(&mut dma, 0, OFF_CNT_H, CTRL_ENABLE_BIT);
        assert_eq!(dma.plan(0).count, 0x4000);
        // DMA3 (16 bits): conteo 0 → 0x10000.
        write_reg16(&mut dma, 3, OFF_CNT_H, CTRL_ENABLE_BIT);
        assert_eq!(dma.plan(3).count, 0x1_0000);
    }

    #[test]
    fn plan_alinea_las_direcciones_a_la_unidad() {
        let mut dma = Dma::new();
        // Direcciones desalineadas con transferencia de 32 bits → se alinean a 4.
        write_reg32(&mut dma, 3, OFF_SAD, 0x0200_0003);
        write_reg32(&mut dma, 3, OFF_DAD, 0x0300_0002);
        write_reg16(&mut dma, 3, OFF_CNT_H, CTRL_WORD_BIT | CTRL_ENABLE_BIT);
        let plan = dma.plan(3);
        assert_eq!(plan.src, 0x0200_0000);
        assert_eq!(plan.dst, 0x0300_0000);
    }

    #[test]
    fn plan_decodifica_pasos_decreciente_y_fijo() {
        let mut dma = Dma::new();
        // dst control = 1 (decrementa), src control = 2 (fija), 16 bits.
        let control = CTRL_ENABLE_BIT
            | (1 << CTRL_DST_CTL_SHIFT)  // dst decrementa
            | (2 << CTRL_SRC_CTL_SHIFT); // src fija
        write_reg16(&mut dma, 1, OFF_CNT_H, control);
        let plan = dma.plan(1);
        assert!(!plan.word, "sin bit 10 → 16 bits");
        assert_eq!(plan.src_step, 0, "origen fijo");
        assert_eq!(plan.dst_step, -2, "destino decrementa una unidad de 16 bits");
    }

    #[test]
    fn finish_immediate_limpia_el_enable_y_el_latch() {
        let mut dma = Dma::new();
        write_reg16(&mut dma, 2, OFF_CNT_H, CTRL_ENABLE_BIT);
        assert!(dma.poll_channel(2)); // flanco → dispararía
        dma.finish_immediate(2);
        // Tras terminar, el control ya no tiene el enable puesto.
        assert_eq!(dma.control(2) & CTRL_ENABLE_BIT, 0);
        // Y un nuevo enable vuelve a ser flanco (re-dispara).
        write_reg16(&mut dma, 2, OFF_CNT_H, CTRL_ENABLE_BIT);
        assert!(dma.poll_channel(2));
    }
}
