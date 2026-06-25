//! **HLE de la BIOS** (*High-Level Emulation*): reimplementación en Rust de las
//! funciones que los juegos invocan por `SWI`, para que el emulador funcione
//! **sin requerir `gba_bios.bin`** (Mini-Hito 2.3a-bis).
//!
//! ## Por qué existe este módulo
//!
//! El Mini-Hito 2.3a dejó la **BIOS real** como opción de fidelidad (LLE, *Low-
//! Level Emulation*: ejecutar el firmware de Nintendo byte a byte). Pero esa BIOS
//! es propietaria y no se distribuye, así que el **camino por defecto** del
//! emulador es el *fallback* sin BIOS. Ahí, un `SWI` saltaría al vector `0x08`,
//! que sin BIOS está a ceros, y la CPU **descarrilaría**.
//!
//! El HLE evita eso: cuando no hay BIOS real ([`crate::Bus::has_bios`] es
//! `false`), el despacho del `SWI` —en [`crate::Cpu`]— **intercepta** la llamada
//! antes de entrar al vector y ejecuta aquí la función nativa equivalente,
//! dejando los resultados en los registros como haría la BIOS de Nintendo. Es
//! exactamente lo que hace mGBA en su modo sin BIOS. Con BIOS real cargada, el
//! `SWI` sigue yendo al vector `0x08` (LLE, Mini-Hito 2.2l) y este módulo no
//! interviene.
//!
//! ## Qué está implementado
//!
//! - **Aritméticas:** [`Div`](div) (0x06), [`DivArm`](div_arm) (0x07),
//!   [`Sqrt`](sqrt) (0x08), [`ArcTan`](arctan) (0x09), [`ArcTan2`](arctan2) (0x0A).
//! - **Memoria:** [`CpuSet`](cpu_set) (0x0B), [`CpuFastSet`](cpu_fast_set) (0x0C).
//! - **Matrices afines:** [`BgAffineSet`](bg_affine_set) (0x0E),
//!   [`ObjAffineSet`](obj_affine_set) (0x0F).
//! - **Descompresión:** [`BitUnPack`](bit_unpack) (0x10),
//!   [`LZ77UnComp`](lz77) (0x11/0x12), [`HuffUnComp`](huff) (0x13),
//!   [`RLUnComp`](rl) (0x14/0x15) y [`Diff*UnFilter`](diff_unfilter) (0x16–0x18).
//! - **Reset/control:** [`SoftReset`](soft_reset) (0x00),
//!   [`RegisterRamReset`](register_ram_reset) (0x01).
//!
//! `Halt` (0x02) ya está implementado (Mini-Hito 2.3c): pone la CPU en bajo
//! consumo hasta que `IE & IF` se hace distinto de cero (ver [`Cpu::halt`]). Desde el
//! Mini-Hito **2.4b**, `IntrWait` (0x04) y `VBlankIntrWait` (0x05) también lo usan:
//! ahora que la PPU genera la IRQ de V-Blank, suspenden la CPU hasta esa interrupción
//! (es una versión simplificada del IntrWait de la BIOS; ver [`dispatch`]). Siguen
//! como **stub** (no-op que solo deja continuar la CPU) `Stop` (0x03) —espera una IRQ
//! de teclado/externa— y las de sonido (0x1A+), que esperan a la APU (Fase 2.5).
//!
//! ## 🛡️ Seguridad — entradas controladas por la ROM
//!
//! Varias funciones (`CpuSet`, las descompresiones) reciben **punteros y
//! longitudes desde registros que la ROM controla**. Por eso **toda** lectura y
//! escritura de memoria se enruta por [`Bus`] —que hace *clamp* y nunca panica— y
//! las longitudes de descompresión se acotan a [`MAX_DECOMP_BYTES`]: una cabecera
//! disparatada o un puntero a memoria no mapeada no puede colgar el emulador ni
//! escribir fuera de sitio.
//!
//! ## Referencia
//!
//! Convenciones de registros y algoritmos según GBATEK; las funciones aritméticas
//! (los coeficientes de `ArcTan`, las ramas de `ArcTan2`, los casos límite de
//! `Div`) y las afines reproducen el HLE de mGBA (`src/gba/bios.c`).

use std::f32::consts::PI;

use crate::bus::{
    Bus, EWRAM_SIZE, EWRAM_START, IWRAM_SIZE, IWRAM_START, OAM_SIZE, OAM_START, PRAM_SIZE,
    PRAM_START, ROM_START, VRAM_SIZE, VRAM_START,
};
use crate::cpu::{Cpu, Executed};

/// Tope de bytes que una descompresión puede producir. Las cabeceras de
/// descompresión llevan un tamaño de 24 bits (hasta 16 MiB) controlado por la
/// ROM; acotarlo a un máximo generoso pero finito evita que una cabecera
/// maliciosa o corrupta dispare un bucle enorme. 4 MiB sobra para cualquier
/// recurso real (la RAM mayor de la GBA, la EWRAM, son 256 KiB).
pub const MAX_DECOMP_BYTES: usize = 4 * 1024 * 1024;

/// Despacha un `SWI` en **modo HLE** por su número de función `number` (ya
/// extraído por el llamante: byte alto del comentario de 24 bits en ARM, `imm8`
/// en THUMB) y ejecuta la función nativa equivalente.
///
/// Devuelve el [`Executed`] que [`Cpu::step`](crate::Cpu) consume: casi todas
/// continúan a la instrucción siguiente al `SWI` ([`Executed::Continue`]); solo
/// [`SoftReset`](soft_reset) salta ([`Executed::Branched`]).
///
/// El **coste en ciclos** de cada `SWI` de BIOS no se modela todavía (se devuelve
/// `extra_cycles: 0`), igual que los *waitstates* de ROM son provisionales hasta
/// emular `WAITCNT`.
/// El **manejador de IRQ** que la BIOS de Nintendo tiene en el vector `0x18`,
/// instalado en la región de BIOS cuando se arranca **sin BIOS real** (modo HLE).
/// Cada par es `(offset dentro de la BIOS, instrucción ARM)`; es el wrapper estándar
/// que la propia consola ejecuta al tomar una IRQ:
///
/// ```text
/// stmfd sp!, {r0-r3, r12, lr}   ; salva el contexto en la pila de IRQ
/// mov   r0, #0x04000000
/// add   lr, pc, #0              ; lr = dirección del ldmfd siguiente
/// ldr   pc, [r0, #-4]           ; salta al manejador de usuario en [0x03FFFFFC]
/// ldmfd sp!, {r0-r3, r12, lr}   ; (al volver) restaura el contexto
/// subs  pc, lr, #4              ; retorna a la instrucción interrumpida
/// ```
///
/// El manejador de usuario lo deja el juego en `0x0300_7FFC` (espejado en
/// `0x03FF_FFFC`). Sin este wrapper, una IRQ tomada (V-Blank de la PPU en 2.4b, un
/// timer, un DMA...) saltaría al vector `0x18` **vacío** (ceros) y la CPU
/// descarrilaría. Es lo que hace usable el sistema de IRQ en el camino por defecto
/// sin `gba_bios.bin`; con BIOS real, [`Bus::load_bios`] sobreescribe esto con el
/// wrapper auténtico.
const HLE_IRQ_WRAPPER: [(usize, u32); 6] = [
    (0x18, 0xE92D_500F),
    (0x1C, 0xE3A0_0301),
    (0x20, 0xE28F_E000),
    (0x24, 0xE510_F004),
    (0x28, 0xE8BD_500F),
    (0x2C, 0xE25E_F004),
];

/// Instala el [`HLE_IRQ_WRAPPER`] en `bios` (la región de BIOS del [`Bus`]). Lo llama
/// [`crate::Bus::new`] para que, sin BIOS real, una IRQ tomada se despache al
/// manejador de usuario en vez de descarrilar en el vector vacío. No toca el resto de
/// la BIOS (el reset y los `SWI` se resuelven por HLE, no por código en `0x0`).
pub(crate) fn install_irq_handler(bios: &mut [u8]) {
    for (off, word) in HLE_IRQ_WRAPPER {
        if let Some(slot) = bios.get_mut(off..off + 4) {
            slot.copy_from_slice(&word.to_le_bytes());
        }
    }
}

pub(crate) fn dispatch(cpu: &mut Cpu, bus: &mut Bus, number: u8) -> Executed {
    match number {
        0x00 => return soft_reset(cpu, bus), // único que salta
        0x01 => register_ram_reset(cpu, bus),
        0x06 => div(cpu),
        0x07 => div_arm(cpu),
        0x08 => sqrt(cpu),
        0x09 => arctan(cpu),
        0x0A => arctan2(cpu),
        0x0B => cpu_set(cpu, bus),
        0x0C => cpu_fast_set(cpu, bus),
        0x0E => bg_affine_set(cpu, bus),
        0x0F => obj_affine_set(cpu, bus),
        0x10 => bit_unpack(cpu, bus),
        0x11 | 0x12 => lz77(cpu, bus, /* write16 */ number == 0x12),
        0x13 => huff(cpu, bus),
        0x14 | 0x15 => rl(cpu, bus, /* write16 */ number == 0x15),
        0x16 => diff_unfilter(cpu, bus, /* unit16 */ false, /* write16 */ false),
        0x17 => diff_unfilter(cpu, bus, /* unit16 */ false, /* write16 */ true),
        0x18 => diff_unfilter(cpu, bus, /* unit16 */ true, /* write16 */ true),
        // `Halt` (0x02): pone la CPU en bajo consumo hasta la próxima IRQ. El
        // mecanismo ya existe (Mini-Hito 2.3c): la CPU despierta cuando `IE & IF`
        // se hace distinto de cero. Continúa a la instrucción siguiente (es esa la
        // que no se ejecutará hasta despertar).
        0x02 => cpu.halt(),
        // `IntrWait` (0x04) / `VBlankIntrWait` (0x05): suspenden la CPU hasta que
        // llega la IRQ esperada. Ahora que la PPU (2.4b) y los timers (2.3e) generan
        // IRQs por tiempo, se implementan reusando el `Halt`: la CPU duerme y el bucle
        // la despierta cuando `IE & IF != 0` (ver [`Cpu::halt`] y
        // [`crate::Bus::next_wakeup_cycle`]). Es una versión **simplificada** —no usa
        // el espejo de `IF` de la BIOS (`0x0300_7FF8`) ni distingue "esperar una
        // nueva" de "ya pendiente"—, suficiente para que un juego que sincroniza con
        // el V-Blank deje de girar en vacío.
        0x04 | 0x05 => cpu.halt(),
        // Stubs aún sin disparador: `Stop` (0x03), que espera una IRQ de teclado/
        // externa, y las de sonido (0x1A+), que esperan a la APU (Fase 2.5). Sin su
        // disparador, lo seguro es no hacer nada y continuar, para no colgar el
        // emulador. Igual para un número no reconocido.
        _ => {}
    }
    Executed::Continue { extra_cycles: 0 }
}

// ===== Aritméticas (SWI 0x06–0x0A) ======================================

/// Núcleo común de `Div`/`DivArm`: división con signo dejando cociente en `r0`,
/// resto en `r1` y `|cociente|` en `r3`. Reproduce los casos límite del HLE de
/// mGBA, que en hardware quedan indefinidos pero conviene tratar para no panicar:
/// - **divisor 0:** `r0 = ±1` (signo del dividendo), `r1 = dividendo`, `r3 = 1`.
/// - **`i32::MIN / -1`** (desbordaría): `r0 = i32::MIN`, `r1 = 0`, `r3 = i32::MIN`.
fn divide(cpu: &mut Cpu, num: i32, denom: i32) {
    let (quot, rem, abs_quot) = if denom == 0 {
        (if num < 0 { -1 } else { 1 }, num, 1)
    } else if denom == -1 && num == i32::MIN {
        (i32::MIN, 0, i32::MIN)
    } else {
        let q = num / denom;
        (q, num % denom, q.wrapping_abs())
    };
    cpu.set_reg(0, quot as u32);
    cpu.set_reg(1, rem as u32);
    cpu.set_reg(3, abs_quot as u32);
}

/// `Div` (SWI 0x06): entrada `r0` = dividendo, `r1` = divisor (con signo).
fn div(cpu: &mut Cpu) {
    let num = cpu.reg(0) as i32;
    let denom = cpu.reg(1) as i32;
    divide(cpu, num, denom);
}

/// `DivArm` (SWI 0x07): idéntica a [`div`] pero con los operandos **al revés**
/// (`r0` = divisor, `r1` = dividendo). Existe por compatibilidad con la ABI de
/// ARM; la salida es la misma.
fn div_arm(cpu: &mut Cpu) {
    let denom = cpu.reg(0) as i32;
    let num = cpu.reg(1) as i32;
    divide(cpu, num, denom);
}

/// `Sqrt` (SWI 0x08): raíz cuadrada entera de `r0` (sin signo), resultado en `r0`.
fn sqrt(cpu: &mut Cpu) {
    cpu.set_reg(0, isqrt(cpu.reg(0)));
}

/// Raíz cuadrada entera (parte entera de √x) por el método dígito-a-dígito en
/// base 4, sin coma flotante (para un resultado determinista y exacto).
fn isqrt(n: u32) -> u32 {
    let mut n = n;
    let mut root: u32 = 0;
    // Mayor potencia de cuatro que cabe en un u32 (4^15 = 2^30).
    let mut bit: u32 = 1 << 30;
    while bit > n {
        bit >>= 2;
    }
    while bit != 0 {
        if n >= root + bit {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    root
}

/// `ArcTan` (SWI 0x09): arcotangente de `r0` (tangente en coma fija 1.14 con
/// signo, 16 bits). Devuelve en `r0` el ángulo en el rango `0xC000..=0x4000`
/// (`-π/2..π/2`). Reproduce el polinomio de la BIOS (coeficientes verbatim de
/// GBATEK/mGBA); la precisión empeora fuera de `±π/4`, como en el hardware.
fn arctan(cpu: &mut Cpu) {
    let i = (cpu.reg(0) as i16) as i32; // tangente con signo, 16 bits
    cpu.set_reg(0, (arctan_core(i) as i16) as u32);
}

/// El polinomio de Horner de `ArcTan`. Toda la aritmética usa `wrapping_*`: en C
/// estos productos se hacen en `int32_t` y dependen del desbordamiento envolvente
/// (definido en la BIOS real); replicarlo evita además cualquier *panic* de
/// desbordamiento en *debug*.
fn arctan_core(i: i32) -> i32 {
    let a = -(i.wrapping_mul(i) >> 14);
    let mut b = (0xA9_i32.wrapping_mul(a) >> 14) + 0x390;
    b = (b.wrapping_mul(a) >> 14) + 0x91C;
    b = (b.wrapping_mul(a) >> 14) + 0xFB6;
    b = (b.wrapping_mul(a) >> 14) + 0x16AA;
    b = (b.wrapping_mul(a) >> 14) + 0x2081;
    b = (b.wrapping_mul(a) >> 14) + 0x3651;
    b = (b.wrapping_mul(a) >> 14) + 0xA2F9;
    i.wrapping_mul(b) >> 16
}

/// `ArcTan2` (SWI 0x0A): arcotangente del vector (`r0` = X, `r1` = Y, ambos con
/// signo de 16 bits) con corrección de cuadrante. Devuelve en `r0` el ángulo en
/// `0x0000..=0xFFFF` (todo el círculo, `0..2π`). La lógica de cuadrantes reproduce
/// la del HLE de mGBA.
fn arctan2(cpu: &mut Cpu) {
    let x = (cpu.reg(0) as i16) as i32;
    let y = (cpu.reg(1) as i16) as i32;
    cpu.set_reg(0, arctan2_core(x, y));
}

/// Núcleo de [`arctan2`]: devuelve el ángulo ya recortado a 16 bits sin signo.
fn arctan2_core(x: i32, y: i32) -> u32 {
    if y == 0 {
        return if x >= 0 { 0x0000 } else { 0x8000 };
    }
    if x == 0 {
        return if y >= 0 { 0x4000 } else { 0xC000 };
    }
    // Mismo árbol de decisión que mGBA: cada rama elige entre ArcTan(Y/X) y
    // ArcTan(X/Y) y le suma el desplazamiento de cuadrante.
    let res: i32 = if y >= 0 {
        if x >= 0 {
            if x >= y {
                arctan_core((y << 14) / x)
            } else {
                0x4000 - arctan_core((x << 14) / y)
            }
        } else if -x >= y {
            arctan_core((y << 14) / x) + 0x8000
        } else {
            0x4000 - arctan_core((x << 14) / y)
        }
    } else if x <= 0 {
        if -x > -y {
            arctan_core((y << 14) / x) + 0x8000
        } else {
            0xC000 - arctan_core((x << 14) / y)
        }
    } else if x >= -y {
        arctan_core((y << 14) / x) + 0x10000
    } else {
        0xC000 - arctan_core((x << 14) / y)
    };
    (res as u32) & 0xFFFF
}

// ===== Memoria (SWI 0x0B–0x0C) ==========================================

/// `CpuSet` (SWI 0x0B): copia o rellena un bloque de memoria. Entrada `r0` =
/// origen, `r1` = destino, `r2` = control:
/// - bits 0-20: número de **unidades** a transferir;
/// - bit 24: modo (0 = copia avanzando el origen, 1 = *fill* con un valor fijo);
/// - bit 26: tamaño de unidad (0 = 16 bits, 1 = 32 bits).
fn cpu_set(cpu: &mut Cpu, bus: &mut Bus) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let ctrl = cpu.reg(2);
    let count = ctrl & 0x1F_FFFF;
    let fixed = (ctrl >> 24) & 1 != 0;
    let word = (ctrl >> 26) & 1 != 0;

    if word {
        let (mut s, mut d) = (src & !3, dst & !3);
        for _ in 0..count {
            bus.write_u32(d, bus.read_u32(s));
            if !fixed {
                s = s.wrapping_add(4);
            }
            d = d.wrapping_add(4);
        }
    } else {
        let (mut s, mut d) = (src & !1, dst & !1);
        for _ in 0..count {
            bus.write_u16(d, bus.read_u16(s));
            if !fixed {
                s = s.wrapping_add(2);
            }
            d = d.wrapping_add(2);
        }
    }
}

/// `CpuFastSet` (SWI 0x0C): como [`cpu_set`] pero **siempre en palabras de 32
/// bits** y en bloques de 8 (el número de unidades se redondea hacia arriba al
/// múltiplo de 8). Entrada `r0`/`r1`/`r2` igual; aquí el bit 26 se ignora y solo
/// importa el bit 24 (*fill*).
fn cpu_fast_set(cpu: &mut Cpu, bus: &mut Bus) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let ctrl = cpu.reg(2);
    let count = (ctrl & 0x1F_FFFF).wrapping_add(7) & !7; // redondeo a múltiplo de 8
    let fixed = (ctrl >> 24) & 1 != 0;

    let (mut s, mut d) = (src & !3, dst & !3);
    for _ in 0..count {
        bus.write_u32(d, bus.read_u32(s));
        if !fixed {
            s = s.wrapping_add(4);
        }
        d = d.wrapping_add(4);
    }
}

// ===== Matrices afines (SWI 0x0E–0x0F) ==================================

/// `BgAffineSet` (SWI 0x0E): calcula los parámetros afines `PA/PB/PC/PD` y el
/// punto de referencia de un fondo a partir de centro, escala y ángulo. Entrada
/// `r0` = origen (array de structs de 20 bytes), `r1` = destino (16 bytes/entrada),
/// `r2` = número de entradas. Reproduce el cálculo en coma flotante del HLE de
/// mGBA (no es bit-exacto con la tabla de senos de la BIOS, pero sí funcional).
fn bg_affine_set(cpu: &mut Cpu, bus: &mut Bus) {
    let mut src = cpu.reg(0);
    let mut dst = cpu.reg(1);
    let count = cpu.reg(2);

    for _ in 0..count {
        let ox = (bus.read_u32(src) as i32) as f32 / 256.0;
        let oy = (bus.read_u32(src.wrapping_add(4)) as i32) as f32 / 256.0;
        let cx = (bus.read_u16(src.wrapping_add(8)) as i16) as f32;
        let cy = (bus.read_u16(src.wrapping_add(10)) as i16) as f32;
        let sx = (bus.read_u16(src.wrapping_add(12)) as i16) as f32 / 256.0;
        let sy = (bus.read_u16(src.wrapping_add(14)) as i16) as f32 / 256.0;
        let theta = (bus.read_u16(src.wrapping_add(16)) >> 8) as f32 / 128.0 * PI;
        src = src.wrapping_add(20);

        let (sin, cos) = theta.sin_cos();
        let a = cos * sx;
        let b = -sin * sx;
        let c = sin * sy;
        let d = cos * sy;
        let rx = ox - (a * cx + b * cy);
        let ry = oy - (c * cx + d * cy);

        bus.write_u16(dst, fixed_u16(a));
        bus.write_u16(dst.wrapping_add(2), fixed_u16(b));
        bus.write_u16(dst.wrapping_add(4), fixed_u16(c));
        bus.write_u16(dst.wrapping_add(6), fixed_u16(d));
        bus.write_u32(dst.wrapping_add(8), (rx * 256.0) as i32 as u32);
        bus.write_u32(dst.wrapping_add(12), (ry * 256.0) as i32 as u32);
        dst = dst.wrapping_add(16);
    }
}

/// `ObjAffineSet` (SWI 0x0F): calcula `PA/PB/PC/PD` de sprites a partir de escala
/// y ángulo. Entrada `r0` = origen (structs de 8 bytes), `r1` = destino, `r2` =
/// número de entradas, `r3` = **separación en bytes** entre cada parámetro escrito
/// (los parámetros de la OAM van intercalados con otros datos del sprite).
fn obj_affine_set(cpu: &mut Cpu, bus: &mut Bus) {
    let mut src = cpu.reg(0);
    let mut dst = cpu.reg(1);
    let count = cpu.reg(2);
    let diff = cpu.reg(3);

    for _ in 0..count {
        let sx = (bus.read_u16(src) as i16) as f32 / 256.0;
        let sy = (bus.read_u16(src.wrapping_add(2)) as i16) as f32 / 256.0;
        let theta = (bus.read_u16(src.wrapping_add(4)) >> 8) as f32 / 128.0 * PI;
        src = src.wrapping_add(8);

        let (sin, cos) = theta.sin_cos();
        let a = cos * sx;
        let b = -sin * sx;
        let c = sin * sy;
        let d = cos * sy;

        bus.write_u16(dst, fixed_u16(a));
        bus.write_u16(dst.wrapping_add(diff), fixed_u16(b));
        bus.write_u16(dst.wrapping_add(diff.wrapping_mul(2)), fixed_u16(c));
        bus.write_u16(dst.wrapping_add(diff.wrapping_mul(3)), fixed_u16(d));
        dst = dst.wrapping_add(diff.wrapping_mul(4));
    }
}

/// Convierte un parámetro afín en coma flotante a coma fija 1.7.8 (×256) y lo
/// recorta a 16 bits, como hacen las dos funciones afines al almacenar `PA..PD`.
fn fixed_u16(value: f32) -> u16 {
    (value * 256.0) as i32 as u16
}

// ===== Descompresión (SWI 0x10–0x18) ====================================

/// Tamaño de datos descomprimidos declarado en la cabecera estándar (común a
/// LZ77/Huffman/RL/Diff): los bits 8-31 de la primera palabra en el origen. Se
/// **acota** a [`MAX_DECOMP_BYTES`] por seguridad (la ROM controla este campo).
fn decompressed_size(bus: &Bus, src: u32) -> usize {
    ((bus.read_u32(src) >> 8) as usize).min(MAX_DECOMP_BYTES)
}

/// Vuelca el buffer descomprimido `out` en `dst`, byte a byte (`write16 == false`)
/// o en media-palabras (`write16 == true`). Los destinos de VRAM no admiten
/// escrituras de 8 bits, de ahí las variantes "write16" de las funciones de
/// descompresión: se acumulan dos bytes y se escriben juntos.
fn flush(bus: &mut Bus, dst: u32, out: &[u8], write16: bool) {
    if write16 {
        let mut i = 0;
        while i + 1 < out.len() {
            let half = (out[i] as u16) | ((out[i + 1] as u16) << 8);
            bus.write_u16(dst.wrapping_add(i as u32), half);
            i += 2;
        }
        if i < out.len() {
            // Byte impar final: se escribe como media-palabra con el alto a 0.
            bus.write_u16(dst.wrapping_add(i as u32), out[i] as u16);
        }
    } else {
        for (i, &byte) in out.iter().enumerate() {
            bus.write_u8(dst.wrapping_add(i as u32), byte);
        }
    }
}

/// `LZ77UnComp` (SWI 0x11 escribe 8 bits / 0x12 escribe 16 bits): descompresión
/// LZ77. Tras la cabecera, un **byte de flags** precede a 8 bloques; cada bit
/// (de MSB a LSB) indica literal (0: copia un byte) o referencia (1: dos bytes
/// que codifican longitud `(b0>>4)+3` y distancia `((b0&0xF)<<8 | b1)+1` hacia
/// atrás en lo ya descomprimido).
fn lz77(cpu: &mut Cpu, bus: &mut Bus, write16: bool) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let size = decompressed_size(bus, src);

    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut sp = src.wrapping_add(4); // datos tras la cabecera
    while out.len() < size {
        let flags = bus.read_u8(sp);
        sp = sp.wrapping_add(1);
        for bit in 0..8 {
            if out.len() >= size {
                break;
            }
            let compressed = (flags >> (7 - bit)) & 1 != 0;
            if compressed {
                let b0 = bus.read_u8(sp);
                let b1 = bus.read_u8(sp.wrapping_add(1));
                sp = sp.wrapping_add(2);
                let length = (b0 >> 4) as usize + 3;
                let disp = (((b0 & 0xF) as usize) << 8) | b1 as usize;
                // Posición de origen de la copia: `disp + 1` bytes hacia atrás.
                let mut from = out.len().wrapping_sub(disp + 1);
                for _ in 0..length {
                    if out.len() >= size {
                        break;
                    }
                    // `get` devuelve 0 si la referencia apunta fuera (stream
                    // corrupto): defensa, en un stream válido `from < out.len()`.
                    let byte = out.get(from).copied().unwrap_or(0);
                    out.push(byte);
                    from = from.wrapping_add(1);
                }
            } else {
                let byte = bus.read_u8(sp);
                sp = sp.wrapping_add(1);
                out.push(byte);
            }
        }
    }
    flush(bus, dst, &out, write16);
}

/// `RLUnComp` (SWI 0x14 escribe 8 bits / 0x15 escribe 16 bits): *run-length*.
/// Tras la cabecera, cada byte de flag indica: bit 7 = 1 → bloque comprimido de
/// `(flag&0x7F)+3` repeticiones del byte siguiente; bit 7 = 0 → bloque de
/// `(flag&0x7F)+1` bytes literales.
fn rl(cpu: &mut Cpu, bus: &mut Bus, write16: bool) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let size = decompressed_size(bus, src);

    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut sp = src.wrapping_add(4);
    while out.len() < size {
        let flag = bus.read_u8(sp);
        sp = sp.wrapping_add(1);
        if flag & 0x80 != 0 {
            let run = (flag & 0x7F) as usize + 3;
            let byte = bus.read_u8(sp);
            sp = sp.wrapping_add(1);
            for _ in 0..run {
                if out.len() >= size {
                    break;
                }
                out.push(byte);
            }
        } else {
            let run = (flag & 0x7F) as usize + 1;
            for _ in 0..run {
                if out.len() >= size {
                    break;
                }
                out.push(bus.read_u8(sp));
                sp = sp.wrapping_add(1);
            }
        }
    }
    flush(bus, dst, &out, write16);
}

/// `Diff8bitUnFilter`/`Diff16bitUnFilter` (SWI 0x16/0x17/0x18): *un-filtering* de
/// diferencias. El stream guarda diferencias respecto al elemento anterior; se
/// reconstruye acumulando (`out[i] = out[i-1] + data[i]`, con el primer elemento
/// relativo a 0). `unit16` elige unidad de 8 o 16 bits; `write16` elige el ancho
/// de escritura (las variantes de VRAM escriben en media-palabras).
fn diff_unfilter(cpu: &mut Cpu, bus: &mut Bus, unit16: bool, write16: bool) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let size = decompressed_size(bus, src); // en bytes
    let mut sp = src.wrapping_add(4);
    let mut out: Vec<u8> = Vec::with_capacity(size);

    if unit16 {
        let mut acc: u16 = 0;
        for _ in 0..size / 2 {
            acc = acc.wrapping_add(bus.read_u16(sp));
            sp = sp.wrapping_add(2);
            out.push(acc as u8);
            out.push((acc >> 8) as u8);
        }
    } else {
        let mut acc: u8 = 0;
        for _ in 0..size {
            acc = acc.wrapping_add(bus.read_u8(sp));
            sp = sp.wrapping_add(1);
            out.push(acc);
        }
    }
    flush(bus, dst, &out, write16);
}

/// `HuffUnComp` (SWI 0x13): descompresión Huffman. Cabecera estándar (con el
/// tamaño de dato en bits 0-3), seguida de un **árbol** y un flujo de bits que se
/// recorre desde la raíz hasta una hoja por cada símbolo. Entrada `r0` = origen,
/// `r1` = destino.
fn huff(cpu: &mut Cpu, bus: &mut Bus) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let header = bus.read_u32(src);
    let size = (header >> 8).min(MAX_DECOMP_BYTES as u32) as usize; // bytes de salida
    let data_bits = header & 0xF; // tamaño de símbolo en bits (4 u 8)
    if data_bits == 0 {
        return;
    }

    // Árbol: en `src+4` está su tamaño en (valor+1)·2 bytes; los nodos empiezan en
    // `src+5`. Cada nodo no-hoja tiene un byte de offset+flags; los bits 6/7
    // marcan si los hijos derecho/izquierdo son hojas.
    let tree_base = src.wrapping_add(4);
    let mut stream = tree_base.wrapping_add((bus.read_u8(tree_base) as u32 + 1) * 2);

    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut pending: u32 = 0; // símbolos a medio formar (nibbles para data_bits=4)
    let mut pending_bits: u32 = 0;

    // Recorre el árbol bit a bit. `node` apunta al nodo actual.
    let mut node = tree_base.wrapping_add(1);
    let mut node_byte = bus.read_u8(node);
    // Cota de seguridad: un árbol corrupto que nunca alcance una hoja no debe
    // colgar el bucle. Cada palabra del flujo produce, en datos válidos, al menos
    // un símbolo; este tope holgado corta el caso patológico.
    let max_words = size.saturating_mul(8).saturating_add(64);
    let mut words_read = 0usize;
    while out.len() < size {
        if words_read >= max_words {
            break;
        }
        let word = bus.read_u32(stream);
        stream = stream.wrapping_add(4);
        words_read += 1;
        for shift in (0..32).rev() {
            if out.len() >= size {
                break;
            }
            let bit = (word >> shift) & 1;
            // Offset al par de hijos: (offset+1)*2 desde la dirección alineada.
            let offset = (node_byte & 0x3F) as u32;
            let next_pair = (node & !1).wrapping_add((offset + 1) * 2);
            let child = next_pair.wrapping_add(bit);
            // ¿El hijo elegido es una hoja? Lo dicen los bits 7 (izq) / 6 (der).
            let leaf_mask = if bit == 0 { 0x80 } else { 0x40 };
            let is_leaf = node_byte & leaf_mask != 0;
            node_byte = bus.read_u8(child);
            if is_leaf {
                // Hoja: `node_byte` es el símbolo. Reensambla la salida según el
                // tamaño de símbolo (4 u 8 bits), poco significativo primero.
                pending |= ((node_byte as u32) & symbol_mask(data_bits)) << pending_bits;
                pending_bits += data_bits;
                while pending_bits >= 8 {
                    out.push(pending as u8);
                    pending >>= 8;
                    pending_bits -= 8;
                }
                node = tree_base.wrapping_add(1);
                node_byte = bus.read_u8(node);
            } else {
                node = child;
            }
        }
    }
    flush(bus, dst, &out, false);
}

/// Máscara de un símbolo de `bits` bits (4 → `0x0F`, 8 → `0xFF`).
fn symbol_mask(bits: u32) -> u32 {
    if bits >= 8 {
        0xFF
    } else {
        (1 << bits) - 1
    }
}

/// `BitUnPack` (SWI 0x10): expande unidades empaquetadas (de 1/2/4/8 bits) a
/// unidades más anchas (1/2/4/8/16/32 bits), opcionalmente sumando un offset.
/// Entrada `r0` = origen, `r1` = destino, `r2` = puntero a la estructura
/// `UnPackInfo` { u16 longitud_origen, u8 ancho_origen, u8 ancho_destino, u32
/// offset (bit 31 = sumar también a las unidades nulas) }.
fn bit_unpack(cpu: &mut Cpu, bus: &mut Bus) {
    let src = cpu.reg(0);
    let dst = cpu.reg(1);
    let info = cpu.reg(2);

    let src_len = bus.read_u16(info) as usize;
    let src_width = bus.read_u8(info.wrapping_add(2)) as u32;
    let dst_width = bus.read_u8(info.wrapping_add(3)) as u32;
    let offset_word = bus.read_u32(info.wrapping_add(4));
    let offset = offset_word & 0x7FFF_FFFF;
    let offset_zero = offset_word & 0x8000_0000 != 0;

    // Anchos válidos según GBATEK; si no, no hacemos nada (defensa).
    if !matches!(src_width, 1 | 2 | 4 | 8) || !matches!(dst_width, 1 | 2 | 4 | 8 | 16 | 32) {
        return;
    }
    let src_mask = (1u32 << src_width) - 1;
    let dst_mask = if dst_width >= 32 { u32::MAX } else { (1u32 << dst_width) - 1 };

    let mut out_buffer: u32 = 0;
    let mut out_bits: u32 = 0;
    let mut out_addr = dst;
    let mut sp = src;

    for _ in 0..src_len {
        let byte = bus.read_u8(sp) as u32;
        sp = sp.wrapping_add(1);
        let mut bitpos = 0;
        while bitpos < 8 {
            let mut unit = (byte >> bitpos) & src_mask;
            bitpos += src_width;
            if unit != 0 || offset_zero {
                unit = unit.wrapping_add(offset);
            }
            out_buffer |= (unit & dst_mask) << out_bits;
            out_bits += dst_width;
            if out_bits >= 32 {
                bus.write_u32(out_addr, out_buffer);
                out_addr = out_addr.wrapping_add(4);
                out_buffer = 0;
                out_bits = 0;
            }
        }
    }
    // Resto parcial (no ocurre con anchos que dividen 32, pero por completitud).
    if out_bits > 0 {
        bus.write_u32(out_addr, out_buffer);
    }
}

// ===== Reset / control (SWI 0x00–0x01) ==================================

/// `SoftReset` (SWI 0x00): reinicia la CPU como la BIOS tras un *soft reset*.
/// Limpia los últimos `0x200` bytes de la IWRAM (área de pila de la BIOS), monta
/// los stack pointers, pasa a modo System en estado ARM y salta a la ROM
/// (`0x0800_0000`) o a la EWRAM (`0x0200_0000`) según el byte de control en
/// `0x0300_7FFA` (0 → ROM). Devuelve [`Executed::Branched`] porque cambia el `PC`.
fn soft_reset(cpu: &mut Cpu, bus: &mut Bus) -> Executed {
    let to_ewram = bus.read_u8(0x0300_7FFA) != 0;
    // Limpia el área alta de la IWRAM (0x03007E00..0x03008000).
    for addr in 0x0300_7E00..0x0300_8000u32 {
        bus.write_u8(addr, 0);
    }
    let entry = if to_ewram { EWRAM_START } else { ROM_START };
    cpu.enter_soft_reset(entry);
    Executed::Branched { extra_cycles: 0 }
}

/// `RegisterRamReset` (SWI 0x01): borra selectivamente regiones de RAM según el
/// mapa de bits en `r0` (bit 0 = EWRAM, 1 = IWRAM salvo sus últimos `0x200`
/// bytes, 2 = PRAM, 3 = VRAM, 4 = OAM).
///
/// Los bits 5-7 (registros de SIO, sonido y resto de I/O) **no** se tratan aún:
/// esos registros todavía son un buffer crudo sin semántica (llega en 2.3c/2.4/
/// 2.5), y borrarlos a ciegas haría más daño que bien.
fn register_ram_reset(cpu: &mut Cpu, bus: &mut Bus) {
    let flags = cpu.reg(0);
    if flags & (1 << 0) != 0 {
        clear_region(bus, EWRAM_START, EWRAM_SIZE);
    }
    if flags & (1 << 1) != 0 {
        // La IWRAM se borra salvo sus últimos 0x200 bytes (pila/IRQ de la BIOS).
        clear_region(bus, IWRAM_START, IWRAM_SIZE - 0x200);
    }
    if flags & (1 << 2) != 0 {
        clear_region(bus, PRAM_START, PRAM_SIZE);
    }
    if flags & (1 << 3) != 0 {
        clear_region(bus, VRAM_START, VRAM_SIZE);
    }
    if flags & (1 << 4) != 0 {
        clear_region(bus, OAM_START, OAM_SIZE);
    }
}

/// Pone a cero `len` bytes desde `start`, a través del bus (que ignora sin
/// panicar lo que caiga fuera de una región).
fn clear_region(bus: &mut Bus, start: u32, len: usize) {
    for i in 0..len as u32 {
        bus.write_u8(start.wrapping_add(i), 0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::CpuMode;

    /// Una CPU + bus de pruebas en modo HLE (sin BIOS), situada en IWRAM y en modo
    /// System (como un juego ya en marcha).
    fn entorno() -> (Cpu, Bus) {
        let mut cpu = Cpu::new();
        cpu.set_mode(CpuMode::System);
        cpu.set_pc(IWRAM_START);
        (cpu, Bus::new(vec![0u8; 64]))
    }

    // ---- Aritméticas ----------------------------------------------------

    #[test]
    fn div_basico_y_casos_limite() {
        let (mut cpu, _) = entorno();
        cpu.set_reg(0, 10);
        cpu.set_reg(1, 3);
        div(&mut cpu);
        assert_eq!(cpu.reg(0), 3, "cociente");
        assert_eq!(cpu.reg(1), 1, "resto");
        assert_eq!(cpu.reg(3), 3, "|cociente|");

        // Negativo: -10 / 3 = -3 (trunca hacia 0), resto -1, |cociente| = 3.
        cpu.set_reg(0, (-10i32) as u32);
        cpu.set_reg(1, 3);
        div(&mut cpu);
        assert_eq!(cpu.reg(0) as i32, -3);
        assert_eq!(cpu.reg(1) as i32, -1);
        assert_eq!(cpu.reg(3), 3);

        // División por cero: r0 = ±1 (signo del dividendo), r1 = dividendo, r3 = 1.
        cpu.set_reg(0, 7);
        cpu.set_reg(1, 0);
        div(&mut cpu);
        assert_eq!(cpu.reg(0), 1);
        assert_eq!(cpu.reg(1), 7);
        assert_eq!(cpu.reg(3), 1);

        // i32::MIN / -1 desbordaría: caso especial documentado.
        cpu.set_reg(0, i32::MIN as u32);
        cpu.set_reg(1, (-1i32) as u32);
        div(&mut cpu);
        assert_eq!(cpu.reg(0), i32::MIN as u32);
        assert_eq!(cpu.reg(1), 0);
        assert_eq!(cpu.reg(3), i32::MIN as u32);
    }

    #[test]
    fn div_arm_invierte_los_operandos() {
        let (mut cpu, _) = entorno();
        cpu.set_reg(0, 3); // divisor
        cpu.set_reg(1, 10); // dividendo
        div_arm(&mut cpu);
        assert_eq!(cpu.reg(0), 3);
        assert_eq!(cpu.reg(1), 1);
    }

    #[test]
    fn sqrt_entero() {
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(1), 1);
        assert_eq!(isqrt(2), 1);
        assert_eq!(isqrt(15), 3);
        assert_eq!(isqrt(16), 4);
        assert_eq!(isqrt(9999), 99);
        assert_eq!(isqrt(0xFFFF_FFFF), 0xFFFF); // 65535² = 0xFFFE0001 ≤ 0xFFFFFFFF
    }

    #[test]
    fn arctan_de_cero_es_cero() {
        assert_eq!(arctan_core(0), 0);
    }

    #[test]
    fn arctan2_direcciones_cardinales() {
        // +X = 0, +Y = 0x4000 (π/2), -X = 0x8000 (π), -Y = 0xC000 (3π/2).
        assert_eq!(arctan2_core(100, 0), 0x0000);
        assert_eq!(arctan2_core(0, 100), 0x4000);
        assert_eq!(arctan2_core(-100, 0), 0x8000);
        assert_eq!(arctan2_core(0, -100), 0xC000);
        // Diagonal del primer cuadrante (x==y): ~45° = 0x2000.
        let diag = arctan2_core(100, 100);
        assert!((0x1F00..=0x2100).contains(&diag), "≈0x2000, fue {diag:#06X}");
    }

    // ---- CpuSet / CpuFastSet -------------------------------------------

    #[test]
    fn cpu_set_copia_32_bits() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        for i in 0..3u32 {
            bus.write_u32(src + i * 4, 0x1000 + i);
        }
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        cpu.set_reg(2, 3 | (1 << 26)); // 3 palabras, 32 bits, copia
        cpu_set(&mut cpu, &mut bus);
        for i in 0..3u32 {
            assert_eq!(bus.read_u32(dst + i * 4), 0x1000 + i);
        }
    }

    #[test]
    fn cpu_set_fill_16_bits() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        bus.write_u16(src, 0xABCD);
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        cpu.set_reg(2, 4 | (1 << 24)); // 4 medias-palabras, 16 bits, fill
        cpu_set(&mut cpu, &mut bus);
        for i in 0..4u32 {
            assert_eq!(bus.read_u16(dst + i * 2), 0xABCD, "rellena con el valor fijo");
        }
    }

    #[test]
    fn cpu_fast_set_redondea_a_multiplo_de_8() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        for i in 0..8u32 {
            bus.write_u32(src + i * 4, i);
        }
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        cpu.set_reg(2, 3); // pide 3 → debe redondear a 8 palabras
        cpu_fast_set(&mut cpu, &mut bus);
        for i in 0..8u32 {
            assert_eq!(bus.read_u32(dst + i * 4), i, "copia las 8 palabras del bloque");
        }
    }

    // ---- Descompresión --------------------------------------------------

    /// Escribe la cabecera estándar de descompresión (tipo + tamaño) en `addr`.
    fn cabecera(bus: &mut Bus, addr: u32, tipo: u32, size: u32) {
        bus.write_u32(addr, (size << 8) | (tipo << 4));
    }

    #[test]
    fn lz77_literales_y_referencia() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        // "AAAA": un literal 'A' y luego una referencia (len 3, distancia 1).
        cabecera(&mut bus, src, 1, 4);
        bus.write_u8(src + 4, 0x40); // flags: bit7=0 (literal), bit6=1 (ref), resto 0
        bus.write_u8(src + 5, 0x41); // literal 'A'
        bus.write_u8(src + 6, 0x00); // ref: len=(0>>4)+3=3
        bus.write_u8(src + 7, 0x00); // disp=0 → 1 byte atrás
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        lz77(&mut cpu, &mut bus, false);
        for i in 0..4u32 {
            assert_eq!(bus.read_u8(dst + i), 0x41, "byte {i}");
        }
    }

    #[test]
    fn rl_comprimido_y_literal() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        // "AAAAA": un bloque comprimido de 5 (=2+3) repeticiones de 'A'.
        cabecera(&mut bus, src, 3, 5);
        bus.write_u8(src + 4, 0x80 | 2); // comprimido, run 2+3=5
        bus.write_u8(src + 5, 0x41); // 'A'
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        rl(&mut cpu, &mut bus, false);
        for i in 0..5u32 {
            assert_eq!(bus.read_u8(dst + i), 0x41);
        }
    }

    #[test]
    fn diff8_reconstruye_acumulando() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        // diferencias [10, 3, 0, 7] → acumulado [10, 13, 13, 20].
        cabecera(&mut bus, src, 8, 4);
        for (i, d) in [10u8, 3, 0, 7].iter().enumerate() {
            bus.write_u8(src + 4 + i as u32, *d);
        }
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        diff_unfilter(&mut cpu, &mut bus, false, false);
        assert_eq!(
            [bus.read_u8(dst), bus.read_u8(dst + 1), bus.read_u8(dst + 2), bus.read_u8(dst + 3)],
            [10, 13, 13, 20]
        );
    }

    #[test]
    fn bit_unpack_1bit_a_8bit() {
        let (mut cpu, mut bus) = entorno();
        let (src, dst, info) = (IWRAM_START, IWRAM_START + 0x100, IWRAM_START + 0x200);
        bus.write_u8(src, 0b1011_0001); // 8 bits → 8 bytes
        // UnPackInfo: longitud 1 byte, ancho origen 1, ancho destino 8, offset 0.
        bus.write_u16(info, 1);
        bus.write_u8(info + 2, 1);
        bus.write_u8(info + 3, 8);
        bus.write_u32(info + 4, 0);
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        cpu.set_reg(2, info);
        bit_unpack(&mut cpu, &mut bus);
        // Bits de menos a más significativo: 1,0,0,0,1,1,0,1.
        assert_eq!(
            (0..8).map(|i| bus.read_u8(dst + i)).collect::<Vec<_>>(),
            vec![1, 0, 0, 0, 1, 1, 0, 1]
        );
    }

    #[test]
    fn huff_8bit_arbol_simple() {
        // Árbol Huffman mínimo: la raíz tiene dos hojas, '0' → 0xAA, '1' → 0xBB.
        // Tree: [size=1][root=0x00 (offset0, ambos hijos hoja)][0xAA][0xBB].
        let (mut cpu, mut bus) = entorno();
        let (src, dst) = (IWRAM_START, IWRAM_START + 0x100);
        // Cabecera: tipo 2 (Huffman), 2 bytes de salida y símbolo de 8 bits en el
        // nibble bajo (bits 0-3) de la misma palabra de cabecera.
        bus.write_u32(src, (2u32 << 8) | (2 << 4) | 8);
        bus.write_u8(src + 4, 1); // tamaño del árbol
        bus.write_u8(src + 5, 0xC0); // raíz: offset 0, bits 6/7 = hijos hoja
        bus.write_u8(src + 6, 0xAA); // hoja izquierda (bit 0)
        bus.write_u8(src + 7, 0xBB); // hoja derecha (bit 1)
        // Flujo de bits: 0 luego 1 (MSB primero) → 0xAA, 0xBB.
        bus.write_u32(src + 8, 0b0100_0000u32 << 24);
        cpu.set_reg(0, src);
        cpu.set_reg(1, dst);
        huff(&mut cpu, &mut bus);
        assert_eq!(bus.read_u8(dst), 0xAA);
        assert_eq!(bus.read_u8(dst + 1), 0xBB);
    }

    // ---- Reset ----------------------------------------------------------

    #[test]
    fn soft_reset_salta_a_la_rom_por_defecto() {
        let (mut cpu, mut bus) = entorno();
        let efecto = soft_reset(&mut cpu, &mut bus);
        assert!(matches!(efecto, Executed::Branched { .. }));
        assert_eq!(cpu.pc(), ROM_START, "flag 0 → arranca en la ROM");
        assert_eq!(cpu.mode(), CpuMode::System, "queda en modo System");
        assert!(!cpu.cpsr().thumb(), "estado ARM");
    }

    #[test]
    fn register_ram_reset_borra_solo_lo_pedido() {
        let (mut cpu, mut bus) = entorno();
        bus.write_u32(EWRAM_START, 0xDEAD_BEEF);
        bus.write_u32(PRAM_START, 0x1234_5678);
        cpu.set_reg(0, 1 << 0); // solo EWRAM
        register_ram_reset(&mut cpu, &mut bus);
        assert_eq!(bus.read_u32(EWRAM_START), 0, "la EWRAM se borró");
        assert_eq!(bus.read_u32(PRAM_START), 0x1234_5678, "la PRAM se conserva");
    }
}
