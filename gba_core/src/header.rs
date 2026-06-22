//! Cabecera del cartucho de GBA.
//!
//! La GBA reserva los primeros `0xC0` (192) bytes de la ROM para una cabecera
//! con metadatos: punto de entrada, logo de Nintendo, título del juego, código
//! de juego, checksum, etc. (mapa completo en GBATEK).
//!
//! Aquí parseamos, de momento, los dos campos de texto del Mini-Hito 1.2b:
//! - **Título** (`0xA0`–`0xAB`, 12 bytes): nombre interno en ASCII mayúsculas.
//! - **Código de juego** (`0xAC`–`0xAF`, 4 bytes): identificador del título.
//!
//! 🛡️ Seguridad: usamos `slice.get(rango)` (que devuelve `Option`) en vez de
//! indexar directamente, y `String::from_utf8_lossy` en vez de asumir ASCII
//! válido, para que una cabecera corrupta o truncada no haga panicar el parser.

/// Offset del título del juego dentro de la ROM.
pub const TITLE_OFFSET: usize = 0xA0;
/// Longitud del título, en bytes (`0xA0`..`0xAC`).
pub const TITLE_LEN: usize = 12;

/// Offset del código de juego dentro de la ROM.
pub const GAME_CODE_OFFSET: usize = 0xAC;
/// Longitud del código de juego, en bytes (`0xAC`..`0xB0`).
pub const GAME_CODE_LEN: usize = 4;

/// Metadatos de la cabecera del cartucho (subconjunto parseado por ahora).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    /// Título interno del juego (p. ej. `"POKEMON FIRE"`).
    pub title: String,
    /// Código de juego de 4 caracteres (p. ej. `"BPRE"`).
    pub game_code: String,
}

impl Header {
    /// Parsea la cabecera a partir de los bytes de la ROM.
    ///
    /// Devuelve `None` si la ROM es demasiado corta para contener los campos.
    /// (Tras la validación de tamaño de [`crate::Cartridge`] esto no ocurre,
    /// pero lo tratamos igualmente en vez de indexar a ciegas.)
    pub fn parse(rom: &[u8]) -> Option<Header> {
        let title = rom.get(TITLE_OFFSET..TITLE_OFFSET + TITLE_LEN)?;
        let game_code = rom.get(GAME_CODE_OFFSET..GAME_CODE_OFFSET + GAME_CODE_LEN)?;
        Some(Header {
            title: decode_text(title),
            game_code: decode_text(game_code),
        })
    }
}

/// Convierte un campo de texto de la cabecera en un `String` legible: tolera
/// bytes no-ASCII (`from_utf8_lossy`) y recorta el relleno de NUL y espacios
/// del final, que es como la GBA rellena los títulos más cortos de 12 bytes.
fn decode_text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes)
        .trim_end_matches(|c: char| c == '\0' || c.is_whitespace())
        .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Construye una ROM mínima (`0xC0` bytes) con `title` y `code` en su sitio.
    fn rom_con(title: &[u8], code: &[u8]) -> Vec<u8> {
        let mut rom = vec![0u8; 0xC0];
        rom[TITLE_OFFSET..TITLE_OFFSET + title.len()].copy_from_slice(title);
        rom[GAME_CODE_OFFSET..GAME_CODE_OFFSET + code.len()].copy_from_slice(code);
        rom
    }

    #[test]
    fn parsea_titulo_y_codigo() {
        let rom = rom_con(b"POKEMON FIRE", b"BPRE");
        let h = Header::parse(&rom).unwrap();
        assert_eq!(h.title, "POKEMON FIRE");
        assert_eq!(h.game_code, "BPRE");
    }

    #[test]
    fn recorta_el_relleno_de_nul_del_titulo() {
        let rom = rom_con(b"MARIO", b"AMRE"); // "MARIO" + 7 bytes NUL
        let h = Header::parse(&rom).unwrap();
        assert_eq!(h.title, "MARIO");
    }

    #[test]
    fn no_panica_con_bytes_no_ascii_en_el_titulo() {
        let rom = rom_con(&[0xFF, 0xFE, 0x80, 0x90], b"AMRE");
        let h = Header::parse(&rom).unwrap(); // lo importante: no panica
        assert_eq!(h.game_code, "AMRE");
    }

    #[test]
    fn devuelve_none_si_la_rom_no_cubre_la_cabecera() {
        let rom = vec![0u8; TITLE_OFFSET]; // ni siquiera llega al título
        assert!(Header::parse(&rom).is_none());
    }
}
