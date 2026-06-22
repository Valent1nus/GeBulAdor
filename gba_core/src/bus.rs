//! El **bus de memoria** de la GBA: el mapa de memoria y el ÚNICO punto por el
//! que la CPU (y más adelante DMA/PPU) leen y escriben.
//!
//! ## El mapa de memoria (resumen)
//!
//! La GBA direcciona 32 bits, pero solo unas pocas franjas están "mapeadas" a
//! memoria real. El **byte alto** de la dirección (`addr >> 24`) selecciona la
//! región, lo que hace el *dispatch* muy cómodo:
//!
//! | Región | Dirección base | Tamaño | Notas |
//! |---|---|---|---|
//! | BIOS   | `0x0000_0000` | 16 KiB | solo lectura; se carga en el Hito 2.3a |
//! | EWRAM  | `0x0200_0000` | 256 KiB | RAM de trabajo externa (más lenta) |
//! | IWRAM  | `0x0300_0000` | 32 KiB | RAM interna (rápida) |
//! | I/O    | `0x0400_0000` | 1 KiB | registros de hardware |
//! | PRAM   | `0x0500_0000` | 1 KiB | paletas de color |
//! | VRAM   | `0x0600_0000` | 96 KiB | memoria de vídeo |
//! | OAM    | `0x0700_0000` | 1 KiB | atributos de sprites |
//! | ROM    | `0x0800_0000` | ≤32 MiB | el cartucho (3 espejos por waitstate) |
//! | SRAM   | `0x0E00_0000` | 64 KiB | guardado del cartucho (Fase 3) |
//!
//! ## 🛡️ El Bus como única línea de defensa
//!
//! Toda lectura/escritura pasa por aquí, así que es el sitio natural para
//! centralizar la regla de seguridad "nunca panicar con una dirección rara".
//! Una ROM corrupta, o un bug en nuestro propio *decode*, pueden generar
//! direcciones arbitrarias (p. ej. tras saltar a un puntero mal calculado). Por
//! eso **ninguna** dirección hace panicar: las regiones no mapeadas devuelven un
//! valor seguro de *open bus* ([`OPEN_BUS`]) y las escrituras a memoria de solo
//! lectura se ignoran.
//!
//! ## ⚠️ Rotación en accesos desalineados (la otra trampa del plan)
//!
//! El ARM7TDMI **no falla** ante una lectura de 32 bits desde una dirección que
//! no es múltiplo de 4: lee la palabra alineada y **rota** el resultado según
//! los bits bajos de la dirección (igual con halfwords y direcciones impares).
//! Ese comportamiento lo modela aquí [`Bus::read_u32`]/[`Bus::read_u16`]; ver el
//! comentario de cada uno. Ignorar esto "funciona por accidente" en pruebas
//! simples y falla de forma muy confusa con ROMs reales que lo aprovechan.

/// Valor devuelto al leer una dirección no mapeada (*open bus*). El hardware
/// real devuelve patrones más complejos, pero `0` es seguro y suficiente por
/// ahora (el plan admite `0` o `0xFF`).
const OPEN_BUS: u8 = 0;

/// Dirección base de la BIOS.
pub const BIOS_START: u32 = 0x0000_0000;
/// Dirección base de la EWRAM (On-board Work RAM).
pub const EWRAM_START: u32 = 0x0200_0000;
/// Dirección base de la IWRAM (On-chip Work RAM).
pub const IWRAM_START: u32 = 0x0300_0000;
/// Dirección base de los registros de I/O.
pub const IO_START: u32 = 0x0400_0000;
/// Dirección base de la Palette RAM.
pub const PRAM_START: u32 = 0x0500_0000;
/// Dirección base de la VRAM.
pub const VRAM_START: u32 = 0x0600_0000;
/// Dirección base de la OAM.
pub const OAM_START: u32 = 0x0700_0000;
/// Dirección base de la ROM del cartucho (primer espejo, waitstate 0).
pub const ROM_START: u32 = 0x0800_0000;
/// Dirección base de la SRAM (memoria de guardado del cartucho).
pub const SRAM_START: u32 = 0x0E00_0000;

/// Tamaño de la BIOS: 16 KiB.
pub const BIOS_SIZE: usize = 16 * 1024;
/// Tamaño de la EWRAM: 256 KiB.
pub const EWRAM_SIZE: usize = 256 * 1024;
/// Tamaño de la IWRAM: 32 KiB.
pub const IWRAM_SIZE: usize = 32 * 1024;
/// Tamaño del bloque de registros de I/O que modelamos: 1 KiB.
pub const IO_SIZE: usize = 0x400;
/// Tamaño de la Palette RAM: 1 KiB.
pub const PRAM_SIZE: usize = 1024;
/// Tamaño de la VRAM: 96 KiB.
pub const VRAM_SIZE: usize = 96 * 1024;
/// Tamaño de la OAM: 1 KiB.
pub const OAM_SIZE: usize = 1024;

/// El bus de memoria con todas las regiones direccionables.
///
/// Cada región es un `Vec<u8>` de su tamaño real. La ROM es la única de tamaño
/// variable (la del cartucho cargado). En fases posteriores se le añadirán la
/// SRAM/Flash/EEPROM de guardado y la lógica real de los registros de I/O.
pub struct Bus {
    bios: Vec<u8>,
    ewram: Vec<u8>,
    iwram: Vec<u8>,
    io: Vec<u8>,
    pram: Vec<u8>,
    vram: Vec<u8>,
    oam: Vec<u8>,
    rom: Vec<u8>,
}

impl Bus {
    /// Crea un bus con todas las RAM internas a cero y la `rom` dada en su sitio.
    ///
    /// La BIOS queda a cero por ahora; el Mini-Hito 2.3a la cargará de verdad
    /// para arrancar como el hardware real.
    pub fn new(rom: Vec<u8>) -> Self {
        Bus {
            bios: vec![0; BIOS_SIZE],
            ewram: vec![0; EWRAM_SIZE],
            iwram: vec![0; IWRAM_SIZE],
            io: vec![0; IO_SIZE],
            pram: vec![0; PRAM_SIZE],
            vram: vec![0; VRAM_SIZE],
            oam: vec![0; OAM_SIZE],
            rom,
        }
    }

    /// Acceso de solo lectura a los bytes de la ROM cargada.
    pub fn rom(&self) -> &[u8] {
        &self.rom
    }

    // ---- Lecturas -------------------------------------------------------

    /// Lee un byte de cualquier dirección. Es la operación primitiva: las
    /// lecturas de 16 y 32 bits se construyen sobre esta.
    ///
    /// Nunca panica: una dirección fuera de toda región mapeada devuelve
    /// [`OPEN_BUS`].
    pub fn read_u8(&self, addr: u32) -> u8 {
        match addr >> 24 {
            // BIOS: no tiene espejo; por encima de 16 KiB es open bus.
            0x00 => read_at(&self.bios, (addr & 0x00FF_FFFF) as usize),
            // EWRAM/IWRAM/PRAM/OAM tienen tamaño potencia de dos y se espejan
            // por toda su franja: basta enmascarar con (tamaño - 1).
            0x02 => read_at(&self.ewram, (addr as usize) & (EWRAM_SIZE - 1)),
            0x03 => read_at(&self.iwram, (addr as usize) & (IWRAM_SIZE - 1)),
            // I/O: por ahora un buffer crudo; la semántica real de cada registro
            // llega en hitos posteriores (PPU, SIO, timers...).
            0x04 => read_at(&self.io, (addr & 0x00FF_FFFF) as usize),
            0x05 => read_at(&self.pram, (addr as usize) & (PRAM_SIZE - 1)),
            // VRAM tiene un espejo peculiar (no es potencia de dos): ver vram_offset.
            0x06 => read_at(&self.vram, vram_offset(addr)),
            0x07 => read_at(&self.oam, (addr as usize) & (OAM_SIZE - 1)),
            // ROM: los waitstates 0/1/2 (0x08..0x0D) son tres espejos de la
            // misma ROM. Enmascarar a 32 MiB los unifica; más allá del tamaño
            // real del cartucho, open bus.
            0x08..=0x0D => read_at(&self.rom, (addr & 0x01FF_FFFF) as usize),
            // 0x01 (hueco), 0x0E/0x0F (SRAM, aún sin implementar) y el resto:
            // open bus. La SRAM llegará en la Fase 3 (guardado).
            _ => OPEN_BUS,
        }
    }

    /// Lee 16 bits en little-endian.
    ///
    /// ⚠️ Si `addr` es **impar**, el ARM7TDMI rota el halfword 8 bits a la
    /// derecha (comportamiento de `LDRH` desalineado), en vez de fallar.
    pub fn read_u16(&self, addr: u32) -> u16 {
        let aligned = addr & !1;
        let lo = self.read_u8(aligned) as u16;
        let hi = self.read_u8(aligned + 1) as u16;
        let value = lo | (hi << 8);
        value.rotate_right((addr & 1) * 8)
    }

    /// Lee 32 bits en little-endian.
    ///
    /// ⚠️ Si `addr` no es múltiplo de 4, se lee la palabra alineada y se **rota**
    /// a la derecha `(addr & 3) * 8` bits (comportamiento de `LDR` desalineado).
    /// Por eso esta función toma la dirección *cruda*, no la alineada.
    pub fn read_u32(&self, addr: u32) -> u32 {
        let aligned = addr & !3;
        let b0 = self.read_u8(aligned) as u32;
        let b1 = self.read_u8(aligned + 1) as u32;
        let b2 = self.read_u8(aligned + 2) as u32;
        let b3 = self.read_u8(aligned + 3) as u32;
        let value = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
        value.rotate_right((addr & 3) * 8)
    }

    // ---- Escrituras -----------------------------------------------------

    /// Escribe un byte. Las regiones de solo lectura (BIOS, ROM) y las no
    /// mapeadas ignoran la escritura sin panicar.
    pub fn write_u8(&mut self, addr: u32, value: u8) {
        match addr >> 24 {
            // BIOS: solo lectura.
            0x00 => {}
            0x02 => write_at(&mut self.ewram, (addr as usize) & (EWRAM_SIZE - 1), value),
            0x03 => write_at(&mut self.iwram, (addr as usize) & (IWRAM_SIZE - 1), value),
            0x04 => write_at(&mut self.io, (addr & 0x00FF_FFFF) as usize, value),
            0x05 => write_at(&mut self.pram, (addr as usize) & (PRAM_SIZE - 1), value),
            0x06 => write_at(&mut self.vram, vram_offset(addr), value),
            0x07 => write_at(&mut self.oam, (addr as usize) & (OAM_SIZE - 1), value),
            // ROM: solo lectura (las escrituras del juego a esta franja son para
            // hardware del cartucho como EEPROM/Flash, que se tratará en Fase 3).
            0x08..=0x0D => {}
            // SRAM y demás: se ignoran por ahora.
            _ => {}
        }
    }

    /// Escribe 16 bits en little-endian. A diferencia de la lectura, una
    /// escritura desalineada **no rota**: simplemente alinea la dirección (es el
    /// comportamiento de `STRH`).
    pub fn write_u16(&mut self, addr: u32, value: u16) {
        let aligned = addr & !1;
        self.write_u8(aligned, value as u8);
        self.write_u8(aligned + 1, (value >> 8) as u8);
    }

    /// Escribe 32 bits en little-endian. Como `STR`, alinea la dirección a
    /// múltiplo de 4 y no rota.
    pub fn write_u32(&mut self, addr: u32, value: u32) {
        let aligned = addr & !3;
        self.write_u8(aligned, value as u8);
        self.write_u8(aligned + 1, (value >> 8) as u8);
        self.write_u8(aligned + 2, (value >> 16) as u8);
        self.write_u8(aligned + 3, (value >> 24) as u8);
    }
}

/// Traduce una dirección de la franja VRAM (`0x06xx_xxxx`) a un offset dentro de
/// los 96 KiB reales, modelando su espejo peculiar: la región se repite cada
/// 128 KiB, y dentro de cada bloque los últimos 32 KiB (`0x18000`–`0x1FFFF`) son
/// un espejo de los `0x10000`–`0x17FFF` anteriores.
fn vram_offset(addr: u32) -> usize {
    let mut offset = (addr & 0x1_FFFF) as usize; // espejo cada 128 KiB
    if offset >= 0x18000 {
        offset -= 0x8000; // los 32 KiB altos reflejan los 32 KiB previos
    }
    offset
}

/// Lee un byte de un buffer en `offset`, devolviendo [`OPEN_BUS`] si el offset
/// cae fuera (p. ej. ROM más pequeña que su franja, o BIOS por encima de su
/// tamaño). Usa `get()` en vez de indexar para no panicar nunca.
#[inline]
fn read_at(buf: &[u8], offset: usize) -> u8 {
    buf.get(offset).copied().unwrap_or(OPEN_BUS)
}

/// Escribe un byte en un buffer en `offset`, ignorando la escritura si cae fuera
/// del buffer (defensa en profundidad: nunca panica por un offset inesperado).
#[inline]
fn write_at(buf: &mut [u8], offset: usize, value: u8) {
    if let Some(slot) = buf.get_mut(offset) {
        *slot = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bus de pruebas con una ROM de 1 KiB de bytes incrementales (0,1,2,...).
    fn bus_de_prueba() -> Bus {
        let rom: Vec<u8> = (0..1024).map(|i| i as u8).collect();
        Bus::new(rom)
    }

    #[test]
    fn ewram_lee_lo_que_escribe() {
        let mut bus = bus_de_prueba();
        bus.write_u32(EWRAM_START, 0x1234_5678);
        assert_eq!(bus.read_u32(EWRAM_START), 0x1234_5678);
    }

    #[test]
    fn la_ewram_se_espeja_cada_256_kib() {
        let mut bus = bus_de_prueba();
        bus.write_u8(EWRAM_START, 0xAB);
        // 0x02040000 es el primer espejo (256 KiB más arriba).
        assert_eq!(bus.read_u8(EWRAM_START + EWRAM_SIZE as u32), 0xAB);
    }

    #[test]
    fn la_iwram_se_espeja_cada_32_kib() {
        let mut bus = bus_de_prueba();
        bus.write_u8(IWRAM_START, 0xCD);
        assert_eq!(bus.read_u8(IWRAM_START + IWRAM_SIZE as u32), 0xCD);
    }

    #[test]
    fn la_vram_refleja_los_32_kib_altos() {
        let mut bus = bus_de_prueba();
        // Escribir en 0x06010000 (offset 0x10000) debe verse en 0x06018000
        // (offset 0x18000), que es su espejo.
        bus.write_u8(VRAM_START + 0x10000, 0x42);
        assert_eq!(bus.read_u8(VRAM_START + 0x18000), 0x42);
    }

    #[test]
    fn escritura_little_endian_correcta() {
        let mut bus = bus_de_prueba();
        bus.write_u32(IWRAM_START, 0xAABB_CCDD);
        // Byte menos significativo primero.
        assert_eq!(bus.read_u8(IWRAM_START), 0xDD);
        assert_eq!(bus.read_u8(IWRAM_START + 1), 0xCC);
        assert_eq!(bus.read_u8(IWRAM_START + 2), 0xBB);
        assert_eq!(bus.read_u8(IWRAM_START + 3), 0xAA);
    }

    #[test]
    fn lee_la_rom_y_devuelve_open_bus_pasado_su_tamano() {
        let bus = bus_de_prueba(); // ROM de 1 KiB con bytes 0,1,2,...
        assert_eq!(bus.read_u8(ROM_START), 0);
        assert_eq!(bus.read_u8(ROM_START + 5), 5);
        // Más allá del tamaño real de la ROM: open bus, sin panicar.
        assert_eq!(bus.read_u8(ROM_START + 4096), OPEN_BUS);
    }

    #[test]
    fn los_waitstates_de_rom_son_espejos() {
        let bus = bus_de_prueba();
        // 0x08, 0x0A y 0x0C apuntan al mismo offset de ROM.
        let v0 = bus.read_u8(0x0800_0007);
        let v1 = bus.read_u8(0x0A00_0007);
        let v2 = bus.read_u8(0x0C00_0007);
        assert_eq!(v0, 7);
        assert_eq!(v1, 7);
        assert_eq!(v2, 7);
    }

    #[test]
    fn la_rom_y_la_bios_ignoran_escrituras() {
        let mut bus = bus_de_prueba();
        bus.write_u8(ROM_START, 0xFF); // debe ignorarse
        assert_eq!(bus.read_u8(ROM_START), 0); // sigue el byte original
        bus.write_u8(BIOS_START, 0xFF);
        assert_eq!(bus.read_u8(BIOS_START), 0);
    }

    #[test]
    fn las_regiones_no_mapeadas_devuelven_open_bus_sin_panicar() {
        let mut bus = bus_de_prueba();
        // 0x01000000 es un hueco; 0x0E000000 es SRAM no implementada.
        assert_eq!(bus.read_u8(0x0100_0000), OPEN_BUS);
        assert_eq!(bus.read_u32(0x0E00_0000), 0);
        bus.write_u32(0x0100_0000, 0xDEAD_BEEF); // no debe panicar
        assert_eq!(bus.read_u32(0x0100_0000), 0);
    }

    #[test]
    fn lectura_u32_desalineada_rota_el_word() {
        let mut bus = bus_de_prueba();
        // Palabra 0xAABBCCDD en una dirección alineada.
        bus.write_u32(IWRAM_START, 0xAABB_CCDD);
        // Leerla con offset +1 rota 8 bits a la derecha: 0xDDAABBCC.
        assert_eq!(bus.read_u32(IWRAM_START + 1), 0xDDAA_BBCC);
        // Offset +2 rota 16 bits: 0xCCDDAABB.
        assert_eq!(bus.read_u32(IWRAM_START + 2), 0xCCDD_AABB);
        // Offset +3 rota 24 bits: 0xBBCCDDAA.
        assert_eq!(bus.read_u32(IWRAM_START + 3), 0xBBCC_DDAA);
    }

    #[test]
    fn lectura_u16_desalineada_rota_el_halfword() {
        let mut bus = bus_de_prueba();
        bus.write_u16(IWRAM_START, 0xBEEF);
        // Dirección impar: el halfword rota 8 bits → 0xEFBE.
        assert_eq!(bus.read_u16(IWRAM_START + 1), 0xEFBE);
    }

    #[test]
    fn lectura_u32_alineada_no_rota() {
        let mut bus = bus_de_prueba();
        bus.write_u32(IWRAM_START, 0x1122_3344);
        assert_eq!(bus.read_u32(IWRAM_START), 0x1122_3344);
    }
}
