//! Cartucho de GBA: carga y **validación** de la ROM.
//!
//! Este módulo es el punto donde el núcleo decide si un blob de bytes es un
//! cartucho aceptable, ANTES de fiarse de él. El frontend lee el archivo del
//! disco (eso es I/O específico de cada plataforma) y nos entrega los bytes;
//! aquí aplicamos la "Regla 2" de seguridad del plan: **validar el tamaño antes
//! de leer/usar nada**.

use crate::header::Header;

/// Tamaño máximo del espacio de cartucho direccionable por una GBA real: 32 MiB
/// (`0x0200_0000`). Cualquier ROM mayor no cabe en el mapa de memoria del
/// hardware, así que la rechazamos en vez de arriesgar asignaciones enormes o
/// accesos fuera del rango previsto más adelante.
pub const MAX_ROM_SIZE: usize = 32 * 1024 * 1024;

/// Tamaño mínimo razonable de una ROM: la cabecera del cartucho ocupa los
/// primeros `0xC0` (192) bytes. Por debajo de eso ni siquiera cabe una cabecera
/// válida, así que un archivo más pequeño es basura o está truncado.
pub const MIN_ROM_SIZE: usize = 0xC0;

/// Motivo por el que un blob de bytes no se acepta como cartucho válido.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CartridgeError {
    /// El archivo es más pequeño que una cabecera de cartucho ([`MIN_ROM_SIZE`]).
    TooSmall { size: usize },
    /// El archivo excede el máximo direccionable por la GBA ([`MAX_ROM_SIZE`]).
    TooLarge { size: usize },
}

impl std::fmt::Display for CartridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CartridgeError::TooSmall { size } => write!(
                f,
                "el archivo es demasiado pequeño ({size} bytes); una cabecera de cartucho \
                 ocupa al menos {MIN_ROM_SIZE} bytes"
            ),
            CartridgeError::TooLarge { size } => write!(
                f,
                "el archivo es demasiado grande ({size} bytes); el máximo direccionable por \
                 la GBA es {MAX_ROM_SIZE} bytes (32 MiB)"
            ),
        }
    }
}

impl std::error::Error for CartridgeError {}

/// Un cartucho de GBA.
///
/// Contiene la ROM ya validada y su cabecera ya parseada. En fases posteriores
/// albergará también la memoria de guardado (SRAM/Flash/EEPROM).
pub struct Cartridge {
    rom: Vec<u8>,
    header: Header,
}

// `Debug` manual a propósito: NO derivamos `#[derive(Debug)]` porque eso
// volcaría los hasta 16-32 MiB de la ROM en cualquier mensaje de pánico o log.
// Mostramos solo lo útil al depurar: título, código de juego y tamaño.
impl std::fmt::Debug for Cartridge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Cartridge")
            .field("title", &self.header.title)
            .field("game_code", &self.header.game_code)
            .field("rom_len", &self.rom.len())
            .finish()
    }
}

impl Cartridge {
    /// Construye un cartucho a partir de los bytes crudos de un `.gba`,
    /// **validando el tamaño antes de aceptarlo** y parseando la cabecera.
    ///
    /// Devuelve [`CartridgeError`] (en vez de panicar) si el archivo es
    /// demasiado pequeño o demasiado grande, para que un `.gba` corrupto o
    /// malicioso no pueda tumbar el emulador.
    pub fn from_bytes(rom: Vec<u8>) -> Result<Self, CartridgeError> {
        let size = rom.len();
        if size < MIN_ROM_SIZE {
            return Err(CartridgeError::TooSmall { size });
        }
        if size > MAX_ROM_SIZE {
            return Err(CartridgeError::TooLarge { size });
        }

        // En este punto `size >= MIN_ROM_SIZE` (0xC0), y la cabecera ocupa
        // justo los primeros 0xC0 bytes, así que `Header::parse` tiene
        // garantizados todos sus bytes. El `expect` documenta esa invariante;
        // no es un panic sobre datos no validados, sino sobre algo que la
        // comprobación de tamaño de arriba acaba de asegurar.
        let header = Header::parse(&rom)
            .expect("la validación de tamaño garantiza la presencia de la cabecera (0xC0 bytes)");

        Ok(Cartridge { rom, header })
    }

    /// Tamaño de la ROM en bytes.
    pub fn len(&self) -> usize {
        self.rom.len()
    }

    /// `true` si la ROM no tiene bytes. (Nunca ocurre tras `from_bytes`, que
    /// exige al menos [`MIN_ROM_SIZE`]; existe porque Clippy lo pide junto a
    /// `len`.)
    pub fn is_empty(&self) -> bool {
        self.rom.is_empty()
    }

    /// Acceso de solo lectura a los bytes crudos de la ROM.
    pub fn rom(&self) -> &[u8] {
        &self.rom
    }

    /// Consume el cartucho y devuelve los bytes de la ROM, para cedérselos al
    /// bus sin clonar los (hasta 32 MiB) de datos. Se usa al montar la consola
    /// en [`crate::Gba::with_cartridge`].
    pub fn into_rom(self) -> Vec<u8> {
        self.rom
    }

    /// Cabecera ya parseada del cartucho (título, código de juego...).
    pub fn header(&self) -> &Header {
        &self.header
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rechaza_archivos_demasiado_pequenos() {
        let err = Cartridge::from_bytes(vec![0; 10]).unwrap_err();
        assert_eq!(err, CartridgeError::TooSmall { size: 10 });
    }

    #[test]
    fn rechaza_archivos_demasiado_grandes() {
        let size = MAX_ROM_SIZE + 1;
        let err = Cartridge::from_bytes(vec![0; size]).unwrap_err();
        assert_eq!(err, CartridgeError::TooLarge { size });
    }

    #[test]
    fn acepta_el_tamano_minimo_exacto() {
        let cart = Cartridge::from_bytes(vec![0; MIN_ROM_SIZE]).expect("debería aceptarse");
        assert_eq!(cart.len(), MIN_ROM_SIZE);
        assert!(!cart.is_empty());
    }

    #[test]
    fn acepta_el_tamano_maximo_exacto() {
        let cart = Cartridge::from_bytes(vec![0; MAX_ROM_SIZE]).expect("debería aceptarse");
        assert_eq!(cart.len(), MAX_ROM_SIZE);
    }

    #[test]
    fn conserva_los_bytes_de_la_rom() {
        let mut datos = vec![0u8; MIN_ROM_SIZE];
        datos[0] = 0xAB;
        datos[MIN_ROM_SIZE - 1] = 0xCD;
        let cart = Cartridge::from_bytes(datos).unwrap();
        assert_eq!(cart.rom()[0], 0xAB);
        assert_eq!(cart.rom()[MIN_ROM_SIZE - 1], 0xCD);
    }

    #[test]
    fn expone_la_cabecera_parseada() {
        // ROM mínima con un título y código conocidos en sus offsets.
        let mut datos = vec![0u8; MIN_ROM_SIZE];
        datos[0xA0..0xA0 + 5].copy_from_slice(b"HELLO");
        datos[0xAC..0xAC + 4].copy_from_slice(b"ABCD");
        let cart = Cartridge::from_bytes(datos).unwrap();
        assert_eq!(cart.header().title, "HELLO");
        assert_eq!(cart.header().game_code, "ABCD");
    }
}
