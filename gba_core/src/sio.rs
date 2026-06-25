//! Los registros del **SIO** (Serial Input/Output): el hardware del **Cable Link**
//! de la GBA (Mini-Hito 2.3d).
//!
//! ## Qué es y por qué se monta ahora
//!
//! El Cable Link permite que dos a cuatro consolas intercambien datos por un
//! cable. Toda su lógica real —transferencias síncronas, IRQ de fin, el algoritmo
//! de *Lockstep*— es cosa de la **Fase 4**. Este hito es deliberadamente modesto:
//! montar los **registros** en el bus para **familiarizarse con el hardware antes
//! de necesitarlo**, de modo que escribirlos y leerlos funcione (los juegos los
//! configuran al arrancar aunque no haya cable conectado), pero **sin** ninguna
//! lógica de transferencia todavía.
//!
//! ## Los registros (GBATEK)
//!
//! El significado de los bits depende del **modo** de comunicación, que se
//! selecciona entre [`RCNT`](Sio) y [`SIOCNT`](Sio) (Normal de 8/32 bits,
//! Multiplay de hasta 4 consolas, UART, JOY bus, o GPIO de propósito general). Por
//! eso aquí solo se **almacenan**: interpretar los bits es de la Fase 4.
//!
//! | Dirección | Registro | Rol |
//! |---|---|---|
//! | `0x0400_0120` | `SIODATA32` / `SIOMULTI0` | dato de 32 bits (Normal) o de la consola 0 (Multiplay) |
//! | `0x0400_0122` | `SIOMULTI1` | dato recibido de la consola 1 (Multiplay) |
//! | `0x0400_0124` | `SIOMULTI2` | dato recibido de la consola 2 (Multiplay) |
//! | `0x0400_0126` | `SIOMULTI3` | dato recibido de la consola 3 (Multiplay) |
//! | `0x0400_0128` | `SIOCNT` | control de la transferencia (reloj, velocidad, Start/Busy, IRQ) |
//! | `0x0400_012A` | `SIODATA8` / `SIOMLT_SEND` | dato de 8 bits (Normal) o a enviar (Multiplay) |
//! | `0x0400_0134` | `RCNT` | modo del puerto a alto nivel / E-S de propósito general (GPIO) |
//!
//! ## ⚠️ Sin lógica = el bit Start/Busy no se auto-limpia
//!
//! En el hardware, escribir el bit **Start/Busy** de `SIOCNT` (bit 7) arranca una
//! transferencia y el bit se mantiene a 1 hasta que termina. Sin lógica de
//! transferencia (no hay cable), aquí ese bit simplemente se **almacena**: un juego
//! que hiciera *polling* esperando a que baje no progresaría. No es un problema en
//! esta fase —no se conecta ningún cable— y lo resolverá la lógica de la Fase 4.
//!
//! ## Reparto con el [`crate::Bus`]
//!
//! Igual que [`crate::dma`] y [`crate::interrupt`], el bus enruta aquí los accesos
//! a estos registros. Este módulo es la **fuente de verdad** de su contenido y el
//! sitio natural donde la Fase 4 añadirá la lógica del Cable Link.

/// Offset (dentro de la región de I/O, base `0x0400_0000`) del primer registro del
/// bloque SIO principal (`SIODATA32`/`SIOMULTI0`).
const SIO_BLOCK_BASE: u32 = 0x120;
/// Fin (exclusivo) del bloque SIO principal: cubre `0x120`–`0x12B` (`SIODATA32`/
/// `SIOMULTI0-3`, `SIOCNT`, `SIODATA8`/`SIOMLT_SEND`).
const SIO_BLOCK_END: u32 = 0x12C;
/// Tamaño en bytes del bloque SIO principal (`0x12C - 0x120`).
const SIO_BLOCK_LEN: usize = (SIO_BLOCK_END - SIO_BLOCK_BASE) as usize;

/// Offset de `RCNT` (16 bits). Está separado del bloque principal (entre medias
/// hay un hueco no usado, `0x12C`–`0x133`).
const RCNT_BASE: u32 = 0x134;
/// Fin (exclusivo) de `RCNT`: `0x134`–`0x135`.
const RCNT_END: u32 = 0x136;

/// Los registros del puerto serie (SIO). Solo **almacenamiento** por ahora; la
/// lógica del Cable Link llega en la Fase 4 (ver el módulo).
pub struct Sio {
    /// Bloque principal `0x120`–`0x12B`, byte a byte y *little-endian* como la
    /// región de I/O: `SIODATA32`/`SIOMULTI0-3`, `SIOCNT` y `SIODATA8`/`SIOMLT_SEND`.
    block: [u8; SIO_BLOCK_LEN],
    /// `RCNT` (`0x134`), los dos bytes en *little-endian*.
    rcnt: [u8; 2],
}

impl Sio {
    /// Crea los registros SIO en reposo (todo a cero), el estado tras un reset.
    pub fn new() -> Self {
        Sio {
            block: [0; SIO_BLOCK_LEN],
            rcnt: [0; 2],
        }
    }

    /// `true` si el offset de I/O `io_off` (los 24 bits bajos de la dirección) cae
    /// en un registro SIO (el bloque principal o `RCNT`). Lo usa el bus para
    /// enrutar aquí el acceso.
    pub fn handles(io_off: u32) -> bool {
        (SIO_BLOCK_BASE..SIO_BLOCK_END).contains(&io_off)
            || (RCNT_BASE..RCNT_END).contains(&io_off)
    }

    /// Lee un byte de un registro SIO. Nunca panica: un offset fuera de los
    /// registros modelados devuelve 0.
    pub fn read_u8(&self, io_off: u32) -> u8 {
        if let Some(i) = block_index(io_off) {
            self.block.get(i).copied().unwrap_or(0)
        } else if let Some(i) = rcnt_index(io_off) {
            self.rcnt.get(i).copied().unwrap_or(0)
        } else {
            0
        }
    }

    /// Escribe un byte en un registro SIO (puro almacenamiento; sin efectos de
    /// transferencia). Nunca panica ante un offset inesperado.
    pub fn write_u8(&mut self, io_off: u32, value: u8) {
        if let Some(i) = block_index(io_off) {
            if let Some(slot) = self.block.get_mut(i) {
                *slot = value;
            }
        } else if let Some(i) = rcnt_index(io_off) {
            if let Some(slot) = self.rcnt.get_mut(i) {
                *slot = value;
            }
        }
    }
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

/// Índice dentro de [`Sio::block`] para un offset de I/O, o `None` si no cae ahí.
fn block_index(io_off: u32) -> Option<usize> {
    (SIO_BLOCK_BASE..SIO_BLOCK_END)
        .contains(&io_off)
        .then(|| (io_off - SIO_BLOCK_BASE) as usize)
}

/// Índice dentro de [`Sio::rcnt`] para un offset de I/O, o `None` si no cae ahí.
fn rcnt_index(io_off: u32) -> Option<usize> {
    (RCNT_BASE..RCNT_END)
        .contains(&io_off)
        .then(|| (io_off - RCNT_BASE) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handles_reconoce_los_registros_sio_y_rcnt() {
        assert!(Sio::handles(0x120)); // SIODATA32/SIOMULTI0
        assert!(Sio::handles(0x128)); // SIOCNT
        assert!(Sio::handles(0x12B)); // último byte de SIODATA8
        assert!(!Sio::handles(0x12C)); // hueco tras el bloque
        assert!(!Sio::handles(0x133)); // sigue en el hueco
        assert!(Sio::handles(0x134)); // RCNT
        assert!(Sio::handles(0x135));
        assert!(!Sio::handles(0x136)); // tras RCNT
    }

    #[test]
    fn siodata32_almacena_y_devuelve_lo_escrito() {
        let mut sio = Sio::new();
        // Escribir SIODATA32 byte a byte (little-endian) y releerlo.
        for (k, b) in [0x78, 0x56, 0x34, 0x12].into_iter().enumerate() {
            sio.write_u8(0x120 + k as u32, b);
        }
        for (k, b) in [0x78, 0x56, 0x34, 0x12].into_iter().enumerate() {
            assert_eq!(sio.read_u8(0x120 + k as u32), b);
        }
    }

    #[test]
    fn siocnt_y_siodata8_son_independientes() {
        let mut sio = Sio::new();
        sio.write_u8(0x128, 0xAB); // SIOCNT byte bajo
        sio.write_u8(0x129, 0xCD); // SIOCNT byte alto
        sio.write_u8(0x12A, 0xEF); // SIODATA8
        assert_eq!(sio.read_u8(0x128), 0xAB);
        assert_eq!(sio.read_u8(0x129), 0xCD);
        assert_eq!(sio.read_u8(0x12A), 0xEF);
    }

    #[test]
    fn rcnt_almacena_y_devuelve_lo_escrito() {
        let mut sio = Sio::new();
        sio.write_u8(0x134, 0x80);
        sio.write_u8(0x135, 0x81);
        assert_eq!(sio.read_u8(0x134), 0x80);
        assert_eq!(sio.read_u8(0x135), 0x81);
    }

    #[test]
    fn un_offset_fuera_de_los_registros_lee_cero_sin_panicar() {
        let mut sio = Sio::new();
        // El hueco entre el bloque y RCNT no es un registro SIO.
        sio.write_u8(0x130, 0xFF); // no se almacena en Sio
        assert_eq!(sio.read_u8(0x130), 0);
    }
}
