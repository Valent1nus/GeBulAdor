//! La **PPU** (Picture Processing Unit): el subsistema gráfico de la GBA
//! (Mini-Hito 2.4a — **Modo 3 Bitmap**).
//!
//! ## Qué hace este hito
//!
//! La PPU es quien convierte el contenido de la memoria de vídeo en los píxeles
//! que ve el jugador. La GBA tiene seis **modos de vídeo** (0–5): tres basados en
//! *tiles* (0, 1, 2) y tres *bitmap* (3, 4, 5). Este primer hito implementa el más
//! sencillo de todos, el **modo 3**, que es la forma más directa de "ver algo en
//! pantalla":
//!
//! - La VRAM (`0x0600_0000`) se interpreta como un **framebuffer crudo** de
//!   240×160 píxeles, **2 bytes por píxel**, fila a fila desde la esquina superior
//!   izquierda. Son 240·160·2 = 76 800 bytes, que caben de sobra en los 96 KiB de
//!   VRAM.
//! - Cada píxel es un color **BGR555** (15 bits): 5 bits por canal, en el orden
//!   `0bX_BBBBB_GGGGG_RRRRR` (rojo en los bits bajos). El bit 15 no se usa.
//!
//! Renderizar el modo 3 es, por tanto, recorrer esos 76 800 bytes y traducir cada
//! color BGR555 al **RGBA8888** que espera el framebuffer del núcleo (ver
//! [`bgr555_to_rgba`]).
//!
//! ## El registro que gobierna el modo: `DISPCNT`
//!
//! La PPU es la **fuente de verdad** de `DISPCNT` (`0x0400_0000`, 16 bits), el
//! registro de control de pantalla. De él, este hito usa dos campos:
//!
//! - **Bits 0-2 — modo de vídeo** (ver [`Ppu::mode`]): de momento solo el 3 dibuja
//!   un bitmap; los demás aún no están implementados.
//! - **Bit 7 — *forced blank*** (ver [`Ppu::forced_blank`]): cuando está a 1, la
//!   PPU desconecta el barrido y la pantalla muestra **blanco** (los juegos lo
//!   activan mientras reconfiguran la VRAM para evitar artefactos).
//!
//! El resto de bits de `DISPCNT` (selección de capas BG/OBJ, ventanas...) se
//! **almacenan** pero todavía no tienen efecto: entran en juego con los fondos
//! (2.4c), los sprites (2.4d) y las ventanas (2.4g).
//!
//! ## Reparto con el [`crate::Bus`]
//!
//! Igual que [`crate::dma`], [`crate::interrupt`], [`crate::sio`] y
//! [`crate::timers`], el bus enruta aquí los accesos a `DISPCNT` y este módulo es su
//! fuente de verdad. La diferencia es que la PPU también **produce una imagen**: la
//! VRAM y la PRAM viven en el bus (es quien gestiona el mapa de memoria y sus
//! espejos), así que el bus se las **presta** a la PPU al renderizar
//! ([`crate::Bus::render_frame`]), y la PPU vuelca el resultado en el framebuffer
//! que le pasa [`crate::Gba::render_frame`].
//!
//! ## Qué queda para los siguientes hitos
//!
//! Este hito renderiza el **frame completo de una vez**, bajo demanda. El paso a
//! **scanlines** con timing de H-Blank/V-Blank —y el flag de V-Blank que desbloquea
//! el arnés de test y los `VBlankIntrWait`— es el Mini-Hito 2.4b, que reutilizará el
//! [`crate::Scheduler`] ya integrado en el bucle. Los modos de *tiles* (0/1/2) son
//! el 2.4c, los sprites el 2.4d, y los modos bitmap 4/5 el 2.4e.

use crate::{BYTES_PER_PIXEL, SCREEN_HEIGHT, SCREEN_WIDTH};

/// Offset (dentro de la región de I/O, base `0x0400_0000`) del registro `DISPCNT`.
const DISPCNT_BASE: u32 = 0x000;
/// Fin (exclusivo) de `DISPCNT`: cubre los dos bytes `0x000`–`0x001`.
const DISPCNT_END: u32 = 0x002;

/// Máscara del **modo de vídeo** en `DISPCNT` (bits 0-2).
const BG_MODE_MASK: u16 = 0b111;
/// Bit de ***forced blank*** en `DISPCNT` (bit 7): pantalla en blanco.
const FORCED_BLANK: u16 = 1 << 7;

/// Color RGBA del *forced blank*: blanco opaco.
const WHITE: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];

/// La unidad de proceso gráfico. Vive dentro del [`crate::Bus`].
///
/// En este hito su estado es únicamente el registro `DISPCNT`; los registros de
/// fondos, scroll, ventanas y efectos se irán sumando en los hitos 2.4b–2.4h.
pub struct Ppu {
    /// Registro de control de pantalla `DISPCNT` (`0x0400_0000`), 16 bits.
    dispcnt: u16,
}

impl Ppu {
    /// Crea la PPU en su estado de reset: `DISPCNT = 0` (modo 0, todo apagado).
    pub fn new() -> Self {
        Ppu { dispcnt: 0 }
    }

    /// `true` si el offset de I/O `io_off` (los 24 bits bajos de la dirección) cae
    /// en un registro que gestiona la PPU. Lo usa el bus para enrutar aquí el
    /// acceso. Por ahora, solo `DISPCNT` (`0x000`–`0x001`).
    pub fn handles(io_off: u32) -> bool {
        (DISPCNT_BASE..DISPCNT_END).contains(&io_off)
    }

    /// Lee un byte de un registro de la PPU. Nunca panica: un offset fuera de los
    /// registros modelados devuelve 0.
    pub fn read_u8(&self, io_off: u32) -> u8 {
        match io_off {
            DISPCNT_BASE => self.dispcnt as u8,
            n if n == DISPCNT_BASE + 1 => (self.dispcnt >> 8) as u8,
            _ => 0,
        }
    }

    /// Escribe un byte en un registro de la PPU. Nunca panica ante un offset
    /// inesperado (simplemente lo ignora).
    pub fn write_u8(&mut self, io_off: u32, value: u8) {
        match io_off {
            DISPCNT_BASE => self.dispcnt = (self.dispcnt & 0xFF00) | u16::from(value),
            n if n == DISPCNT_BASE + 1 => {
                self.dispcnt = (self.dispcnt & 0x00FF) | (u16::from(value) << 8)
            }
            _ => {}
        }
    }

    /// El **modo de vídeo** activo (0–7), de los bits 0-2 de `DISPCNT`. Los valores
    /// 6 y 7 son inválidos en el hardware; de los válidos, este hito solo dibuja el
    /// 3.
    pub fn mode(&self) -> u8 {
        (self.dispcnt & BG_MODE_MASK) as u8
    }

    /// `true` si el bit de ***forced blank*** (bit 7 de `DISPCNT`) está activo: la
    /// pantalla debe mostrarse en blanco.
    pub fn forced_blank(&self) -> bool {
        self.dispcnt & FORCED_BLANK != 0
    }

    /// **Renderiza un frame completo** en `fb` (formato RGBA, [`crate::FRAMEBUFFER_SIZE`]
    /// bytes) a partir de la `vram` y la `pram` que le presta el bus.
    ///
    /// El orden de decisión reproduce el del hardware:
    /// 1. Si está el ***forced blank***, la pantalla es **blanca** y no se mira nada
    ///    más.
    /// 2. Según el modo de vídeo:
    ///    - **Modo 3**: bitmap directo 16bpp desde la VRAM (lo implementado en este
    ///      hito).
    ///    - **Resto de modos** (aún sin implementar): se pinta el **color de fondo**
    ///      (*backdrop*), que es la entrada 0 de la paleta (`PRAM[0]`). Es lo que se
    ///      ve en una pantalla "vacía" del hardware real y la base sobre la que los
    ///      hitos siguientes compondrán las capas.
    pub fn render_frame(&self, vram: &[u8], pram: &[u8], fb: &mut [u8]) {
        if self.forced_blank() {
            fill(fb, WHITE);
            return;
        }
        match self.mode() {
            3 => render_mode3(vram, fb),
            _ => fill(fb, backdrop(pram)),
        }
    }
}

impl Default for Ppu {
    fn default() -> Self {
        Self::new()
    }
}

/// Convierte un color **BGR555** (15 bits, el formato nativo de la GBA) a
/// **RGBA8888** (el del framebuffer del núcleo).
///
/// El empaquetado BGR555 es `0bX_BBBBB_GGGGG_RRRRR`: 5 bits por canal, rojo en los
/// bits bajos, y el bit 15 sin usar. Para escalar cada canal de 5 a 8 bits se
/// replican los bits altos en los bajos (`c8 = (c5 << 3) | (c5 >> 2)`), que reparte
/// los 32 niveles de forma uniforme por el rango 0–255 (0→0, 31→255), en vez de
/// dejar oscuros los máximos como haría un simple desplazamiento. El alfa es siempre
/// `0xFF` (opaco), como el resto del framebuffer.
pub fn bgr555_to_rgba(color: u16) -> [u8; 4] {
    let r5 = (color & 0x1F) as u8;
    let g5 = ((color >> 5) & 0x1F) as u8;
    let b5 = ((color >> 10) & 0x1F) as u8;
    [expand5(r5), expand5(g5), expand5(b5), 0xFF]
}

/// Escala un canal de color de 5 bits (0–31) a 8 bits (0–255) replicando los bits
/// altos en los bajos. Ver [`bgr555_to_rgba`].
#[inline]
fn expand5(c5: u8) -> u8 {
    (c5 << 3) | (c5 >> 2)
}

/// El color de fondo (*backdrop*): la entrada 0 de la paleta, en `PRAM[0..2]`
/// (BGR555 *little-endian*), convertida a RGBA. Lee con `get` para no panicar nunca.
fn backdrop(pram: &[u8]) -> [u8; 4] {
    let color = u16::from_le_bytes([
        pram.first().copied().unwrap_or(0),
        pram.get(1).copied().unwrap_or(0),
    ]);
    bgr555_to_rgba(color)
}

/// Renderiza el **modo 3**: la VRAM como un bitmap directo de 240×160 píxeles a
/// 16bpp. Recorre cada píxel del framebuffer, lee su color BGR555 de la VRAM
/// (2 bytes, *little-endian*) y lo convierte a RGBA. Usa `get` en la VRAM para no
/// panicar nunca, aunque por tamaño (96 KiB) el bitmap completo (76 800 B) siempre
/// cabe.
fn render_mode3(vram: &[u8], fb: &mut [u8]) {
    let pixels = SCREEN_WIDTH * SCREEN_HEIGHT;
    for (i, out) in fb.chunks_exact_mut(BYTES_PER_PIXEL).take(pixels).enumerate() {
        let off = i * 2;
        let color = u16::from_le_bytes([
            vram.get(off).copied().unwrap_or(0),
            vram.get(off + 1).copied().unwrap_or(0),
        ]);
        out.copy_from_slice(&bgr555_to_rgba(color));
    }
}

/// Rellena todo el framebuffer `fb` con un mismo color RGBA.
fn fill(fb: &mut [u8], rgba: [u8; 4]) {
    for out in fb.chunks_exact_mut(BYTES_PER_PIXEL) {
        out.copy_from_slice(&rgba);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{PRAM_SIZE, VRAM_SIZE};
    use crate::FRAMEBUFFER_SIZE;

    #[test]
    fn handles_solo_reconoce_dispcnt() {
        assert!(Ppu::handles(0x000)); // DISPCNT byte bajo
        assert!(Ppu::handles(0x001)); // DISPCNT byte alto
        assert!(!Ppu::handles(0x002)); // ya no es DISPCNT
        assert!(!Ppu::handles(0x004)); // DISPSTAT: aún sin dueño (llega en 2.4b)
    }

    #[test]
    fn dispcnt_almacena_y_devuelve_lo_escrito() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x07); // byte bajo
        ppu.write_u8(0x001, 0x12); // byte alto
        assert_eq!(ppu.read_u8(0x000), 0x07);
        assert_eq!(ppu.read_u8(0x001), 0x12);
    }

    #[test]
    fn mode_extrae_los_bits_0_a_2() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0b1111_1011); // modo = 3, más bits altos a 1
        assert_eq!(ppu.mode(), 3);
    }

    #[test]
    fn forced_blank_detecta_el_bit_7() {
        let mut ppu = Ppu::new();
        assert!(!ppu.forced_blank());
        ppu.write_u8(0x000, FORCED_BLANK as u8);
        assert!(ppu.forced_blank());
    }

    #[test]
    fn conversion_de_color_bgr555_casos_clave() {
        // Negro y blanco (los extremos), y los tres canales puros.
        assert_eq!(bgr555_to_rgba(0x0000), [0x00, 0x00, 0x00, 0xFF]); // negro
        assert_eq!(bgr555_to_rgba(0x7FFF), [0xFF, 0xFF, 0xFF, 0xFF]); // blanco (todo a 31)
        assert_eq!(bgr555_to_rgba(0x001F), [0xFF, 0x00, 0x00, 0xFF]); // rojo puro
        assert_eq!(bgr555_to_rgba(0x03E0), [0x00, 0xFF, 0x00, 0xFF]); // verde puro
        assert_eq!(bgr555_to_rgba(0x7C00), [0x00, 0x00, 0xFF, 0xFF]); // azul puro
        // El bit 15 se ignora: 0x8000 es indistinguible del negro.
        assert_eq!(bgr555_to_rgba(0x8000), [0x00, 0x00, 0x00, 0xFF]);
    }

    #[test]
    fn modo3_vuelca_la_vram_al_framebuffer() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x03); // modo 3
        let mut vram = vec![0u8; VRAM_SIZE];
        let pram = vec![0u8; PRAM_SIZE];
        let mut fb = vec![0u8; FRAMEBUFFER_SIZE];

        // Píxel (0,0) = rojo puro (0x001F); último píxel = azul puro (0x7C00).
        vram[0..2].copy_from_slice(&0x001Fu16.to_le_bytes());
        let last = (SCREEN_WIDTH * SCREEN_HEIGHT - 1) * 2;
        vram[last..last + 2].copy_from_slice(&0x7C00u16.to_le_bytes());

        ppu.render_frame(&vram, &pram, &mut fb);

        assert_eq!(&fb[0..4], &[0xFF, 0x00, 0x00, 0xFF], "primer píxel rojo");
        let n = fb.len();
        assert_eq!(&fb[n - 4..n], &[0x00, 0x00, 0xFF, 0xFF], "último píxel azul");
    }

    #[test]
    fn forced_blank_pinta_todo_blanco() {
        let mut ppu = Ppu::new();
        // Modo 3 con VRAM no nula, pero el forced blank manda: todo blanco.
        ppu.write_u8(0x000, 0x03 | FORCED_BLANK as u8);
        let mut vram = vec![0u8; VRAM_SIZE];
        vram[0..2].copy_from_slice(&0x001Fu16.to_le_bytes()); // rojo (debe ignorarse)
        let pram = vec![0u8; PRAM_SIZE];
        let mut fb = vec![0u8; FRAMEBUFFER_SIZE];

        ppu.render_frame(&vram, &pram, &mut fb);

        assert_eq!(&fb[0..4], &WHITE, "el forced blank ignora la VRAM");
        let n = fb.len();
        assert_eq!(&fb[n - 4..n], &WHITE);
    }

    #[test]
    fn un_modo_no_implementado_pinta_el_backdrop() {
        let mut ppu = Ppu::new();
        ppu.write_u8(0x000, 0x00); // modo 0 (tiles, aún sin implementar)
        let vram = vec![0u8; VRAM_SIZE];
        let mut pram = vec![0u8; PRAM_SIZE];
        pram[0..2].copy_from_slice(&0x03E0u16.to_le_bytes()); // backdrop = verde
        let mut fb = vec![0u8; FRAMEBUFFER_SIZE];

        ppu.render_frame(&vram, &pram, &mut fb);

        // Toda la pantalla es el color de fondo de la paleta.
        assert_eq!(&fb[0..4], &[0x00, 0xFF, 0x00, 0xFF], "backdrop verde");
        let n = fb.len();
        assert_eq!(&fb[n - 4..n], &[0x00, 0xFF, 0x00, 0xFF]);
    }
}
