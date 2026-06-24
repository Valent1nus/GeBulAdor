//! BIOS de la GBA: carga y **validación** del firmware de arranque.
//!
//! La GBA arranca ejecutando su BIOS en `0x0000_0000` (16 KiB de solo lectura),
//! y es la BIOS la que inicializa el hardware y cede el control al cartucho. Este
//! módulo es el punto donde el núcleo decide si un blob de bytes es una BIOS
//! aceptable, ANTES de fiarse de él —igual que [`crate::Cartridge`] hace con la
//! ROM—, aplicando la "Regla 2" de seguridad del plan: **validar el tamaño antes
//! de usar nada**.
//!
//! ## ⚠️ La BIOS es propietaria: no se distribuye con el emulador
//!
//! `gba_bios.bin` es firmware con copyright de Nintendo; **no** se incluye en
//! este repositorio ni se puede descargar desde aquí. El usuario debe aportar su
//! propio volcado (legal si lo extrae de su propia consola). Por eso la BIOS es
//! **opcional** (Mini-Hito 2.3a): si se proporciona, la consola arranca de forma
//! fiel desde `0x0` ([`crate::Gba::with_cartridge_and_bios`]); si no, cae al
//! atajo "skip BIOS" ([`crate::Gba::with_cartridge`] / [`crate::Cpu::skip_bios_init`]).

use crate::bus::BIOS_SIZE;

/// Motivo por el que un blob de bytes no se acepta como BIOS válida.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BiosError {
    /// El archivo no mide exactamente [`BIOS_SIZE`] bytes (16 KiB). La BIOS de la
    /// GBA tiene un tamaño fijo, así que cualquier otro tamaño delata un dump
    /// truncado, corrupto o de otra consola, y se rechaza en vez de cargarlo.
    WrongSize { size: usize },
}

impl std::fmt::Display for BiosError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BiosError::WrongSize { size } => write!(
                f,
                "la BIOS mide {size} bytes; debe medir exactamente {BIOS_SIZE} bytes (16 KiB)"
            ),
        }
    }
}

impl std::error::Error for BiosError {}

/// La BIOS de la GBA: 16 KiB de firmware ya validado, listo para cargar en el
/// bus en `0x0000_0000`.
pub struct Bios {
    bytes: Vec<u8>,
}

// `Debug` manual a propósito: NO derivamos `#[derive(Debug)]` porque eso volcaría
// los 16 KiB de la BIOS en cualquier mensaje de pánico o log. Mostramos solo su
// tamaño, que es lo único útil al depurar.
impl std::fmt::Debug for Bios {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bios").field("len", &self.bytes.len()).finish()
    }
}

impl Bios {
    /// Construye una BIOS a partir de los bytes crudos de un `gba_bios.bin`,
    /// **validando que mida exactamente [`BIOS_SIZE`]** (16 KiB) antes de
    /// aceptarla.
    ///
    /// Devuelve [`BiosError`] (en vez de panicar) si el tamaño no cuadra, para
    /// que un archivo equivocado —otra consola, un dump a medias— no pueda tumbar
    /// el emulador.
    pub fn from_bytes(bytes: Vec<u8>) -> Result<Self, BiosError> {
        let size = bytes.len();
        if size != BIOS_SIZE {
            return Err(BiosError::WrongSize { size });
        }
        Ok(Bios { bytes })
    }

    /// Acceso de solo lectura a los 16 KiB de la BIOS. Lo usa
    /// [`crate::bus::Bus::load_bios`] al volcarla en su región.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Tamaño en bytes (siempre [`BIOS_SIZE`] tras `from_bytes`).
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    /// `true` si no tiene bytes. (Nunca ocurre tras `from_bytes`, que exige
    /// [`BIOS_SIZE`]; existe porque Clippy lo pide junto a `len`.)
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Consume la BIOS y devuelve sus bytes.
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rechaza_un_tamano_menor() {
        let err = Bios::from_bytes(vec![0; BIOS_SIZE - 1]).unwrap_err();
        assert_eq!(err, BiosError::WrongSize { size: BIOS_SIZE - 1 });
    }

    #[test]
    fn rechaza_un_tamano_mayor() {
        let err = Bios::from_bytes(vec![0; BIOS_SIZE + 1]).unwrap_err();
        assert_eq!(err, BiosError::WrongSize { size: BIOS_SIZE + 1 });
    }

    #[test]
    fn acepta_el_tamano_exacto_de_16_kib() {
        let bios = Bios::from_bytes(vec![0; BIOS_SIZE]).expect("16 KiB es válido");
        assert_eq!(bios.len(), BIOS_SIZE);
        assert!(!bios.is_empty());
    }

    #[test]
    fn conserva_los_bytes() {
        let mut datos = vec![0u8; BIOS_SIZE];
        datos[0] = 0x18; // primer byte de la BIOS real (parte de un «b 0x000000D4»)
        datos[BIOS_SIZE - 1] = 0xAB;
        let bios = Bios::from_bytes(datos).unwrap();
        assert_eq!(bios.as_bytes()[0], 0x18);
        assert_eq!(bios.as_bytes()[BIOS_SIZE - 1], 0xAB);
    }
}
