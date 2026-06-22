//! # gba_core — el núcleo del emulador de Game Boy Advance
//!
//! Esta crate es una **librería pura**: no depende de ninguna librería gráfica,
//! de ventanas ni de entrada. Su única salida visual es un *framebuffer* crudo
//! en formato **RGBA** (240 × 160 × 4 bytes) que el frontend (por ejemplo
//! `gba_desktop`) se encarga de pintar en pantalla.
//!
//! Mantener esta separación desde el día 1 es lo que permitirá, más adelante,
//! sustituir el frontend de escritorio por uno de Android, iOS o WASM sin tocar
//! una sola línea del núcleo.
//!
//! ## Estado actual (Fase 2.1a)
//!
//! Además de cargar y validar el cartucho (Fase 1), el núcleo ya tiene el
//! esqueleto del hardware sobre el que correrá la emulación: la CPU ARM7TDMI
//! ([`Cpu`]) con sus registros y modos, y el bus de memoria ([`Bus`]) con el
//! mapa de memoria de la consola. Todavía no se ejecuta ninguna instrucción;
//! eso empieza en los siguientes mini-hitos. La frontera con el frontend
//! —entregar un buffer RGBA— no cambiará.

pub mod bus;
pub mod cartridge;
pub mod cpu;
pub mod header;

pub use bus::Bus;
pub use cartridge::{Cartridge, CartridgeError, MAX_ROM_SIZE, MIN_ROM_SIZE};
pub use cpu::{Cpu, CpuMode, Cpsr};
pub use header::Header;

/// Anchura de la pantalla de la GBA, en píxeles.
pub const SCREEN_WIDTH: usize = 240;

/// Altura de la pantalla de la GBA, en píxeles.
pub const SCREEN_HEIGHT: usize = 160;

/// Bytes por píxel en el framebuffer: un byte para cada canal R, G, B y A.
pub const BYTES_PER_PIXEL: usize = 4;

/// Tamaño total del framebuffer en bytes (240 × 160 × 4 = 153 600).
pub const FRAMEBUFFER_SIZE: usize = SCREEN_WIDTH * SCREEN_HEIGHT * BYTES_PER_PIXEL;

/// Estado completo de una GBA emulada.
///
/// De momento solo contiene el framebuffer. En fases posteriores este será el
/// objeto de nivel superior que agrupe la CPU, el bus de memoria, la PPU, el
/// scheduler, etc. El frontend interactúa con la emulación únicamente a través
/// de este tipo.
pub struct Gba {
    /// Framebuffer en formato RGBA, con [`FRAMEBUFFER_SIZE`] bytes.
    ///
    /// El orden es fila a fila desde la esquina superior izquierda; cada píxel
    /// son 4 bytes consecutivos: `[R, G, B, A]`.
    framebuffer: Vec<u8>,
}

impl Gba {
    /// Crea una nueva GBA con el framebuffer inicializado a un azul sólido.
    ///
    /// El color de arranque es puramente de prueba para la Fase 1.1 ("Hola
    /// Ventana"): demuestra que el núcleo produce píxeles y que el frontend los
    /// pinta. Desaparecerá en cuanto la PPU genere imágenes reales.
    pub fn new() -> Self {
        let mut gba = Gba {
            framebuffer: vec![0; FRAMEBUFFER_SIZE],
        };
        // Azul "GBA" (#1E90FF) como color de prueba visible.
        gba.clear(0x1E, 0x90, 0xFF);
        gba
    }

    /// Rellena todo el framebuffer con un color sólido opaco.
    ///
    /// El canal alfa se fija siempre a `0xFF` (totalmente opaco).
    pub fn clear(&mut self, r: u8, g: u8, b: u8) {
        for pixel in self.framebuffer.chunks_exact_mut(BYTES_PER_PIXEL) {
            pixel[0] = r;
            pixel[1] = g;
            pixel[2] = b;
            pixel[3] = 0xFF;
        }
    }

    /// Devuelve el framebuffer crudo en formato RGBA para que el frontend lo
    /// pinte. Esta es la **única** salida visual del núcleo.
    pub fn framebuffer(&self) -> &[u8] {
        &self.framebuffer
    }
}

impl Default for Gba {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn las_dimensiones_son_las_de_la_gba_real() {
        assert_eq!(SCREEN_WIDTH, 240);
        assert_eq!(SCREEN_HEIGHT, 160);
        assert_eq!(FRAMEBUFFER_SIZE, 240 * 160 * 4);
    }

    #[test]
    fn el_framebuffer_tiene_el_tamano_correcto() {
        let gba = Gba::new();
        assert_eq!(gba.framebuffer().len(), FRAMEBUFFER_SIZE);
    }

    #[test]
    fn clear_rellena_todos_los_pixeles_con_el_color() {
        let mut gba = Gba::new();
        gba.clear(10, 20, 30);
        let fb = gba.framebuffer();

        // Primer píxel.
        assert_eq!(&fb[0..4], &[10, 20, 30, 0xFF]);
        // Último píxel: confirma que el bucle cubre todo el buffer.
        let n = fb.len();
        assert_eq!(&fb[n - 4..n], &[10, 20, 30, 0xFF]);
    }
}
