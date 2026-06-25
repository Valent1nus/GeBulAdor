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
//! | BIOS   | `0x0000_0000` | 16 KiB | solo lectura; cargada en el Hito 2.3a (opcional) |
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
//!
//! ## Registros de I/O con comportamiento: el DMA (Mini-Hito 2.3b)
//!
//! La región de I/O (`0x04xx_xxxx`) es, por ahora, un buffer crudo: leer y
//! escribir un registro solo guarda/devuelve su valor. La **excepción** es el
//! bloque de registros del [`Dma`] (`0x0400_00B0`–`0x0400_00DF`): el bus enruta
//! sus accesos al controlador de DMA y, tras escribir un control con el `enable`,
//! ejecuta la **copia inmediata** (ver [`Bus::poll_dma_triggers`]). Es el primer
//! registro de I/O con semántica real; los timers (2.3e) y el IRQ (2.3c) seguirán
//! el mismo patrón.
//!
//! ## Interrupciones (Mini-Hito 2.3c)
//!
//! El bus alberga también el controlador de interrupciones ([`InterruptControl`]):
//! enruta a él los registros `IE`/`IF`/`IME` y ofrece la API que conecta a los
//! componentes con la CPU — [`Bus::request_interrupt`] (la usa el DMA al terminar,
//! y la usarán timers/PPU), [`Bus::irq_pending`] y [`Bus::irq_raised`] (las
//! consulta la CPU para decidir si salta al vector de IRQ o despierta de un
//! `Halt`).

use crate::dma::{Dma, DMA_CHANNELS};
use crate::interrupt::{Interrupt, InterruptControl};

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

/// Anchura de un acceso a memoria: byte (8), media palabra (16) o palabra (32).
/// La usa [`Bus::access_cycles`] para el conteo de ciclos del Mini-Hito 2.2c.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessWidth {
    /// 8 bits.
    Byte,
    /// 16 bits.
    Half,
    /// 32 bits.
    Word,
}

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

    /// El controlador de **DMA** (Mini-Hito 2.3b): la fuente de verdad de los
    /// registros DMA y de qué copiar. La copia en sí la ejecuta el propio bus (ver
    /// [`Bus::run_dma_transfer`]), porque es quien accede a la memoria.
    dma: Dma,

    /// El controlador de **interrupciones** (Mini-Hito 2.3c): `IE`/`IF`/`IME`. Los
    /// componentes solicitan IRQs por [`Bus::request_interrupt`] y la CPU consulta
    /// [`Bus::irq_pending`]/[`Bus::irq_raised`].
    irq: InterruptControl,

    /// `true` si se ha cargado la **BIOS real** ([`Bus::load_bios`]). Es la fuente
    /// de verdad de "¿hay BIOS?" y decide el camino del `SWI`: con BIOS real se
    /// salta al vector `0x08` (LLE, Mini-Hito 2.2l); sin ella se intercepta y se
    /// ejecuta el **HLE** de la función (Mini-Hito 2.3a-bis). Ver [`Bus::has_bios`].
    bios_loaded: bool,
}

impl Bus {
    /// Crea un bus con todas las RAM internas a cero y la `rom` dada en su sitio.
    ///
    /// La BIOS arranca a cero; si se dispone de `gba_bios.bin` (la BIOS real,
    /// opcional —es propietaria—), [`Bus::load_bios`] la vuelca en su región para
    /// arrancar como el hardware (Mini-Hito 2.3a). Sin ella, la región queda a
    /// cero y la consola usa el atajo "skip BIOS".
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
            dma: Dma::new(),
            irq: InterruptControl::new(),
            bios_loaded: false,
        }
    }

    /// Vuelca el firmware de la BIOS en su región (`0x0`), reemplazando los ceros
    /// con que arranca [`Bus::new`] (Mini-Hito 2.3a). Copia hasta [`BIOS_SIZE`]
    /// bytes sin cambiar el tamaño del buffer: si llegara una BIOS más corta —no
    /// debería, [`crate::Bios`] exige 16 KiB exactos— el resto se queda a cero, y
    /// una más larga se trunca; en ningún caso panica. Lo invoca
    /// [`crate::Gba::with_cartridge_and_bios`].
    pub fn load_bios(&mut self, bios: &[u8]) {
        let n = bios.len().min(BIOS_SIZE);
        self.bios[..n].copy_from_slice(&bios[..n]);
        self.bios_loaded = true;
    }

    /// `true` si se ha cargado la **BIOS real** (firmware de Nintendo) con
    /// [`Bus::load_bios`]. Lo consulta el despacho del `SWI`: con BIOS real, la
    /// llamada va al vector `0x08` para que la ejecute la BIOS (LLE); sin ella,
    /// se intercepta y se ejecuta el **HLE** en Rust (Mini-Hito 2.3a-bis).
    pub fn has_bios(&self) -> bool {
        self.bios_loaded
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
            // I/O: en su mayoría un buffer crudo; la semántica real de cada
            // registro llega en hitos posteriores (PPU, SIO, timers...). La
            // excepción es el bloque DMA (2.3b), que se enruta al controlador.
            0x04 => {
                let off = addr & 0x00FF_FFFF;
                if Dma::in_range(off) {
                    self.dma.read_u8(off)
                } else if InterruptControl::handles(off) {
                    self.irq.read_u8(off)
                } else {
                    read_at(&self.io, off as usize)
                }
            }
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
            // I/O: el bloque DMA se enruta al controlador (solo guarda el byte; el
            // disparo se decide tras la escritura de 16/32 bits). El resto, buffer.
            0x04 => {
                let off = addr & 0x00FF_FFFF;
                if Dma::in_range(off) {
                    self.dma.write_u8(off, value);
                } else if InterruptControl::handles(off) {
                    self.irq.write_u8(off, value);
                } else {
                    write_at(&mut self.io, off as usize, value);
                }
            }
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
        self.after_io_write(aligned, 2);
    }

    /// Escribe 32 bits en little-endian. Como `STR`, alinea la dirección a
    /// múltiplo de 4 y no rota.
    pub fn write_u32(&mut self, addr: u32, value: u32) {
        let aligned = addr & !3;
        self.write_u8(aligned, value as u8);
        self.write_u8(aligned + 1, (value >> 8) as u8);
        self.write_u8(aligned + 2, (value >> 16) as u8);
        self.write_u8(aligned + 3, (value >> 24) as u8);
        self.after_io_write(aligned, 4);
    }

    // ---- DMA (Mini-Hito 2.3b) -------------------------------------------

    /// Gancho tras una escritura de 16/32 bits: si tocó el bloque de registros DMA,
    /// sondea posibles disparos. La detección va aquí (y no en [`Bus::write_u8`])
    /// porque los juegos escriben los registros DMA con accesos de 16/32 bits, y el
    /// disparo depende del control completo (`CNT_H`), no de un byte suelto.
    fn after_io_write(&mut self, aligned_addr: u32, width: u32) {
        if aligned_addr >> 24 == 0x04 && Dma::touches(aligned_addr & 0x00FF_FFFF, width) {
            self.poll_dma_triggers();
        }
    }

    /// Recorre los cuatro canales y ejecuta los que un flanco de `enable` acaba de
    /// disparar en **modo inmediato** ([`Dma::poll_channel`]).
    ///
    /// La **guarda de reentrada** evita que una transferencia que (por una ROM
    /// rara) escriba sobre un registro DMA dispare otra anidada: si ya hay una en
    /// curso, no se sondea nada.
    fn poll_dma_triggers(&mut self) {
        if self.dma.is_running() {
            return;
        }
        for ch in 0..DMA_CHANNELS {
            if self.dma.poll_channel(ch) {
                self.run_dma_transfer(ch);
            }
        }
    }

    /// Ejecuta la copia de un canal de DMA: pide el plan al controlador
    /// ([`Dma::plan`]) y mueve las unidades de `src` a `dst` a través del propio
    /// bus (de ahí que la copia viva aquí y no en [`crate::dma`]).
    ///
    /// Toda lectura/escritura pasa por `read_*`/`write_*`, que ya hacen *clamp* y
    /// nunca panican; el conteo está acotado por el hardware (ver [`crate::dma`]),
    /// así que el bucle no puede dispararse sin control.
    ///
    /// ⚠️ El **coste en ciclos** del DMA todavía **no** se contabiliza: se integrará
    /// cuando el [`crate::Scheduler`] se enchufe al bucle (timers 2.3e, PPU 2.4b).
    /// El **IRQ de fin** (bit 14 del control) tampoco se genera aún: depende del
    /// sistema de interrupciones (2.3c).
    fn run_dma_transfer(&mut self, ch: usize) {
        let plan = self.dma.plan(ch);
        self.dma.begin(); // guarda de reentrada activa durante la copia
        let mut src = plan.src;
        let mut dst = plan.dst;
        for _ in 0..plan.count {
            if plan.word {
                let value = self.read_u32(src);
                self.write_u32(dst, value);
            } else {
                let value = self.read_u16(src);
                self.write_u16(dst, value);
            }
            src = src.wrapping_add(plan.src_step as u32);
            dst = dst.wrapping_add(plan.dst_step as u32);
        }
        let raise_irq = self.dma.irq_on_end(ch); // bit 14 del control, antes de bajarlo
        self.dma.end();
        self.dma.finish_immediate(ch); // inmediato = disparo único: baja el enable
        if raise_irq {
            // El DMA con "IRQ al terminar" levanta la IRQ de su canal (2.3c).
            self.request_interrupt(Interrupt::dma(ch));
        }
    }

    // ---- Interrupciones (Mini-Hito 2.3c) --------------------------------

    /// Marca la IRQ de `source` como pendiente (pone su bit en `IF`). La llaman los
    /// componentes de hardware al disparar: hoy el DMA al terminar; mañana los
    /// timers (2.3e), la PPU (2.4b) y el SIO (Fase 4).
    pub fn request_interrupt(&mut self, source: Interrupt) {
        self.irq.request(source);
    }

    /// `true` si hay alguna IRQ habilitada y pendiente (`IE & IF != 0`), **sin**
    /// mirar `IME`. Es lo que despierta a la CPU de un `Halt` (ver [`crate::Cpu`]).
    pub fn irq_raised(&self) -> bool {
        self.irq.raised()
    }

    /// `true` si una IRQ debe atenderse: [`Bus::irq_raised`] **y** `IME = 1`. La CPU
    /// comprueba además el bit `I` del `CPSR` antes de saltar al vector.
    pub fn irq_pending(&self) -> bool {
        self.irq.pending()
    }

    // ---- Temporización (Mini-Hito 2.2c) ---------------------------------

    /// Ciclos que cuesta un **acceso a memoria** a `addr` con la anchura `width`,
    /// secuencial (`seq` = acceso *S*) o no (acceso *N*). Es la base del conteo
    /// de ciclos: cada región tiene su ancho de bus y sus *waitstates*, y un
    /// acceso de 32 bits a una región de bus de 16 bits cuesta dos sub-accesos
    /// (el segundo siempre secuencial).
    ///
    /// Los tiempos de las regiones fijas (BIOS, IWRAM, I/O, OAM, PRAM, VRAM,
    /// EWRAM) son los del hardware; los de la ROM son **provisionales** (asumen
    /// los waitstates por defecto, ya que `WAITCNT` aún no se emula).
    pub fn access_cycles(&self, addr: u32, width: AccessWidth, seq: bool) -> u32 {
        let t = region_timing(addr);
        let first = 1 + if seq { t.wait_s } else { t.wait_n };
        if width == AccessWidth::Word && t.bus16 {
            first + (1 + t.wait_s)
        } else {
            first
        }
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

/// Parámetros de temporización de una región: ancho de bus y *waitstates* para
/// accesos N (no secuencial) y S (secuencial).
struct RegionTiming {
    /// `true` si la región usa un bus de 16 bits (un acceso de 32 bits cuesta dos
    /// sub-accesos); `false` si es de 32 bits.
    bus16: bool,
    /// Waitstates de un acceso no secuencial (N).
    wait_n: u32,
    /// Waitstates de un acceso secuencial (S).
    wait_s: u32,
}

/// Temporización de la región que contiene `addr` (Mini-Hito 2.2c).
fn region_timing(addr: u32) -> RegionTiming {
    match addr >> 24 {
        // BIOS, IWRAM, I/O, OAM: bus de 32 bits sin waitstates → 1 ciclo.
        0x00 | 0x03 | 0x04 | 0x07 => RegionTiming { bus16: false, wait_n: 0, wait_s: 0 },
        // PRAM y VRAM: bus de 16 bits sin waitstates.
        0x05 | 0x06 => RegionTiming { bus16: true, wait_n: 0, wait_s: 0 },
        // EWRAM: bus de 16 bits con 2 waitstates.
        0x02 => RegionTiming { bus16: true, wait_n: 2, wait_s: 2 },
        // ROM del cartucho: bus de 16 bits; waitstates por defecto (WS0 = 4 para
        // N, 2 para S). PROVISIONAL hasta emular `WAITCNT`.
        0x08..=0x0D => RegionTiming { bus16: true, wait_n: 4, wait_s: 2 },
        // Resto (huecos, SRAM aún sin timing propio): 1 ciclo, conservador.
        _ => RegionTiming { bus16: false, wait_n: 0, wait_s: 0 },
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
    fn has_bios_distingue_el_modo_hle_del_lle() {
        // Sin cargar BIOS, el bus declara que no hay (camino HLE del SWI).
        let mut bus = bus_de_prueba();
        assert!(!bus.has_bios(), "un bus recién creado no tiene BIOS real");
        // Tras cargar una, pasa a modo LLE (el SWI irá al vector 0x08).
        bus.load_bios(&[0u8; BIOS_SIZE]);
        assert!(bus.has_bios(), "load_bios marca que hay BIOS real");
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

    #[test]
    fn ciclos_de_acceso_por_region() {
        use AccessWidth::*;
        let bus = bus_de_prueba();
        // IWRAM: bus de 32 bits, 0 waits → 1 ciclo (S o N, 16 o 32 bits).
        assert_eq!(bus.access_cycles(IWRAM_START, Word, true), 1);
        assert_eq!(bus.access_cycles(IWRAM_START, Word, false), 1);
        // VRAM: bus de 16 bits → 32 bits = 2 sub-accesos; 16 bits = 1.
        assert_eq!(bus.access_cycles(VRAM_START, Word, true), 2);
        assert_eq!(bus.access_cycles(VRAM_START, Half, true), 1);
        // EWRAM: bus de 16 bits, 2 waits → 16b = 3, 32b = 6.
        assert_eq!(bus.access_cycles(EWRAM_START, Half, true), 3);
        assert_eq!(bus.access_cycles(EWRAM_START, Word, true), 6);
        // ROM (WS0 por defecto, provisional): N y S distintos.
        assert_eq!(bus.access_cycles(ROM_START, Half, false), 5); // 1 + 4 (N)
        assert_eq!(bus.access_cycles(ROM_START, Half, true), 3); //  1 + 2 (S)
        assert_eq!(bus.access_cycles(ROM_START, Word, false), 8); // 5 (N) + 3 (S)
        assert_eq!(bus.access_cycles(ROM_START, Word, true), 6); //  3 (S) + 3 (S)
    }

    // ---- DMA (Mini-Hito 2.3b) -------------------------------------------

    /// Dirección de I/O del registro `off` (dentro del canal) del canal `ch`.
    /// `off`: 0=SAD, 4=DAD, 8=CNT_L, 0xA=CNT_H.
    fn dma_reg(ch: u32, off: u32) -> u32 {
        IO_START + 0xB0 + ch * 0x0C + off
    }

    // Bits del control `CNT_H` usados en los tests.
    const DMA_ENABLE: u16 = 1 << 15;
    const DMA_WORD: u16 = 1 << 10; // 32 bits (si no, 16)

    #[test]
    fn dma_copia_inmediata_de_32_bits() {
        // La "Prueba" del hito: copiar un bloque de memoria vía DMA y verificarlo.
        let mut bus = Bus::new(Vec::new());
        // Origen en EWRAM: 4 palabras reconocibles.
        let datos = [0x1111_1111u32, 0x2222_2222, 0x3333_3333, 0x4444_4444];
        for (i, w) in datos.iter().enumerate() {
            bus.write_u32(EWRAM_START + (i as u32) * 4, *w);
        }
        // Programar DMA3: EWRAM → IWRAM, 4 palabras de 32 bits, inmediato.
        bus.write_u32(dma_reg(3, 0), EWRAM_START); // SAD
        bus.write_u32(dma_reg(3, 4), IWRAM_START); // DAD
        bus.write_u16(dma_reg(3, 8), 4); // CNT_L = 4 unidades
        // Escribir el control con enable dispara la copia inmediata aquí mismo.
        bus.write_u16(dma_reg(3, 0xA), DMA_ENABLE | DMA_WORD);

        // El destino contiene ya las 4 palabras.
        for (i, w) in datos.iter().enumerate() {
            assert_eq!(bus.read_u32(IWRAM_START + (i as u32) * 4), *w);
        }
        // Y el enable se ha auto-limpiado (inmediato = disparo único).
        assert_eq!(bus.read_u16(dma_reg(3, 0xA)) & DMA_ENABLE, 0);
    }

    #[test]
    fn dma_disparado_por_una_escritura_de_32_bits_al_control() {
        // Muchos juegos escriben CNT_L+CNT_H de una vez con un STR de 32 bits:
        // el word a CNT_L (0x...B8) cubre también CNT_H y debe disparar.
        let mut bus = Bus::new(Vec::new());
        bus.write_u32(EWRAM_START, 0xCAFE_BABE);
        bus.write_u32(dma_reg(0, 0), EWRAM_START); // SAD
        bus.write_u32(dma_reg(0, 4), IWRAM_START); // DAD
        // count=1 (mitad baja) + control enable|word (mitad alta), en un word.
        let cnt = 1u32 | ((DMA_ENABLE | DMA_WORD) as u32) << 16;
        bus.write_u32(dma_reg(0, 8), cnt);

        assert_eq!(bus.read_u32(IWRAM_START), 0xCAFE_BABE);
    }

    #[test]
    fn dma_copia_de_16_bits_con_origen_fijo() {
        // Origen fijo (control de origen = 2) → rellena el destino con un valor,
        // como hará la FIFO de sonido (que aún no existe). 16 bits.
        let mut bus = Bus::new(Vec::new());
        bus.write_u16(EWRAM_START, 0xABCD);
        bus.write_u32(dma_reg(3, 0), EWRAM_START); // SAD
        bus.write_u32(dma_reg(3, 4), IWRAM_START); // DAD
        bus.write_u16(dma_reg(3, 8), 3); // 3 unidades
        let src_fijo = 2u16 << 7; // control de origen = fija
        bus.write_u16(dma_reg(3, 0xA), DMA_ENABLE | src_fijo); // 16 bits

        // Los tres halfwords del destino son el mismo valor (origen no avanzó).
        assert_eq!(bus.read_u16(IWRAM_START), 0xABCD);
        assert_eq!(bus.read_u16(IWRAM_START + 2), 0xABCD);
        assert_eq!(bus.read_u16(IWRAM_START + 4), 0xABCD);
        // Y no escribió una cuarta unidad.
        assert_eq!(bus.read_u16(IWRAM_START + 6), 0);
    }

    #[test]
    fn los_cuatro_canales_copian() {
        // Cada canal copia una palabra distinta a un destino distinto.
        let mut bus = Bus::new(Vec::new());
        for ch in 0..4u32 {
            let src = EWRAM_START + ch * 4;
            let dst = IWRAM_START + 0x100 + ch * 4;
            let valor = 0x1000_0000 * (ch + 1);
            bus.write_u32(src, valor);
            bus.write_u32(dma_reg(ch, 0), src);
            bus.write_u32(dma_reg(ch, 4), dst);
            bus.write_u16(dma_reg(ch, 8), 1);
            bus.write_u16(dma_reg(ch, 0xA), DMA_ENABLE | DMA_WORD);
            assert_eq!(bus.read_u32(dst), valor, "canal {ch} debe copiar");
        }
    }

    #[test]
    fn un_dma_de_vblank_no_copia_inmediatamente() {
        // Modo de arranque V-Blank (timing 1): queda armado pero NO copia ahora
        // (su disparador llega con la PPU, 2.4b).
        let mut bus = Bus::new(Vec::new());
        bus.write_u32(EWRAM_START, 0xDEAD_BEEF);
        bus.write_u32(dma_reg(3, 0), EWRAM_START);
        bus.write_u32(dma_reg(3, 4), IWRAM_START);
        bus.write_u16(dma_reg(3, 8), 1);
        let vblank = 1u16 << 12;
        bus.write_u16(dma_reg(3, 0xA), DMA_ENABLE | DMA_WORD | vblank);

        // El destino sigue a cero: no se disparó.
        assert_eq!(bus.read_u32(IWRAM_START), 0);
        // Y el canal sigue armado (enable a 1).
        assert_ne!(bus.read_u16(dma_reg(3, 0xA)) & DMA_ENABLE, 0);
    }

    #[test]
    fn dma_con_irq_al_terminar_levanta_la_interrupcion_del_canal() {
        // El bit 14 del control pide IRQ al terminar (Mini-Hito 2.3c): tras la
        // copia, el bus debe levantar la IRQ del canal correspondiente.
        let mut bus = Bus::new(Vec::new());
        bus.write_u32(EWRAM_START, 0x1234_5678);
        bus.write_u32(dma_reg(2, 0), EWRAM_START); // SAD
        bus.write_u32(dma_reg(2, 4), IWRAM_START); // DAD
        bus.write_u16(dma_reg(2, 8), 1);
        const DMA_IRQ: u16 = 1 << 14;
        bus.write_u16(dma_reg(2, 0xA), DMA_ENABLE | DMA_WORD | DMA_IRQ);

        // La copia ocurrió y, además, la IRQ del DMA2 quedó pendiente en IF.
        assert_eq!(bus.read_u32(IWRAM_START), 0x1234_5678);
        let if_bit_dma2 = bus.read_u16(IO_START + 0x202) & Interrupt::Dma2.bit();
        assert_ne!(if_bit_dma2, 0, "el DMA2 con bit 14 levanta su IRQ");
    }

    #[test]
    fn dma_sin_bit_de_irq_no_levanta_interrupcion() {
        let mut bus = Bus::new(Vec::new());
        bus.write_u32(EWRAM_START, 0x1234_5678);
        bus.write_u32(dma_reg(2, 0), EWRAM_START);
        bus.write_u32(dma_reg(2, 4), IWRAM_START);
        bus.write_u16(dma_reg(2, 8), 1);
        bus.write_u16(dma_reg(2, 0xA), DMA_ENABLE | DMA_WORD); // sin bit 14
        assert_eq!(bus.read_u16(IO_START + 0x202), 0, "IF sigue limpio");
    }

    #[test]
    fn dma_con_direcciones_no_mapeadas_no_panica() {
        // 🛡️ Seguridad: origen/destino disparatados (los controla la ROM) no deben
        // colgar el emulador; el bus hace clamp y la copia es inofensiva.
        let mut bus = Bus::new(Vec::new());
        bus.write_u32(dma_reg(3, 0), 0x0100_0000); // origen en un hueco
        bus.write_u32(dma_reg(3, 4), 0x0E00_0000); // destino en SRAM (no implementada)
        bus.write_u16(dma_reg(3, 8), 16);
        bus.write_u16(dma_reg(3, 0xA), DMA_ENABLE | DMA_WORD); // no debe panicar
        // Llegar aquí sin pánico es la prueba.
        assert_eq!(bus.read_u16(dma_reg(3, 0xA)) & DMA_ENABLE, 0);
    }
}
