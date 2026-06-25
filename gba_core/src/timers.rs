//! Los cuatro **temporizadores** de hardware de la GBA (TM0–TM3, Mini-Hito 2.3e).
//!
//! ## Qué son y para qué sirven
//!
//! Cada timer es un contador de 16 bits que **incrementa solo** y, al pasar de
//! `0xFFFF` a `0x10000`, **desborda**: se recarga con un valor configurable y, si
//! se pidió, lanza una **IRQ**. Son la base de dos cosas: una **fuente de tiempo**
//! para los juegos (medir intervalos, generar interrupciones periódicas) y el
//! **ritmo del audio** (la APU vacía sus FIFO al compás de un timer, Fase 2.5).
//!
//! Un timer avanza de una de dos formas:
//! - **Por prescaler** (lo normal): incrementa cada 1, 64, 256 o 1024 ciclos de la
//!   CPU (≈16.78 MHz), según los bits 0-1 del control.
//! - **En cascada** (*count-up*, bit 2): no usa el reloj, sino que incrementa cada
//!   vez que **desborda el timer anterior**. Encadenando dos timers se cuentan
//!   intervalos más largos que los 16 bits de uno solo. No aplica a TM0.
//!
//! ## Apoyo en el [`Scheduler`]: no se decrementa ciclo a ciclo
//!
//! Emular un timer restando 1 a cada ciclo sería lentísimo. En su lugar, como pide
//! el plan, se **calcula cuándo desbordará** y se programa ese instante en el
//! [`Scheduler`]: un timer con `reload` y prescaler `p` desborda dentro de
//! `(0x10000 - reload) · p` ciclos. El valor del contador entre medias se **deduce
//! del tiempo transcurrido** cuando alguien lo lee ([`Timers::read_u8`]), sin
//! mantenerlo al día a cada ciclo. Este es el primer subsistema que **integra el
//! scheduler en el bucle** (ver [`crate::Bus::sync_to_cycle`]).
//!
//! ## Reparto con el [`crate::Bus`]
//!
//! Igual que [`crate::dma`]/[`crate::interrupt`]/[`crate::sio`], el bus enruta aquí
//! los registros (`TM0CNT`–`TM3CNT`, `0x0400_0100`–`0x0400_010F`) y, en su
//! [`sync_to_cycle`](crate::Bus::sync_to_cycle), entrega a este módulo los desbordes
//! que el scheduler dispara, para recargar y, si procede, solicitar la IRQ.

use crate::bus::Event;
use crate::interrupt::{Interrupt, InterruptControl};
use crate::scheduler::Scheduler;

/// Número de timers del hardware (TM0–TM3).
pub const NUM_TIMERS: usize = 4;

/// Ciclos de CPU por incremento según los bits 0-1 del control (el *prescaler*):
/// ÷1, ÷64, ÷256, ÷1024.
const PRESCALER_DIV: [u64; 4] = [1, 64, 256, 1024];

/// Offset (en la región de I/O) del primer registro de timer (`TM0CNT_L`).
const TIMER_IO_BASE: u32 = 0x100;
/// Fin (exclusivo) del bloque de timers: `0x100`–`0x10F` (4 timers × 4 bytes).
const TIMER_IO_END: u32 = 0x110;
/// Bytes que ocupa cada timer: `CNT_L`(2) + `CNT_H`(2).
const TIMER_STRIDE: u32 = 4;

// Bits del control `TMxCNT_H`.
/// Prescaler (bits 0-1): índice en [`PRESCALER_DIV`].
const CTRL_PRESCALER: u16 = 0b11;
/// *Count-up* / cascada (bit 2): incrementa con el desborde del timer anterior.
const CTRL_CASCADE: u16 = 1 << 2;
/// IRQ al desbordar (bit 6).
const CTRL_IRQ: u16 = 1 << 6;
/// Enable (bit 7): el timer está activo.
const CTRL_ENABLE: u16 = 1 << 7;

/// El estado de un timer.
struct Timer {
    /// Valor de **recarga** (`TMxCNT_L` escrito): con él se recarga el contador al
    /// desbordar (y al habilitarse). Escribirlo **no** cambia el contador actual.
    reload: u16,
    /// Registro de control `TMxCNT_H`.
    control: u16,
    /// Valor del contador en el ciclo [`Timer::anchor`]. Para un timer por
    /// prescaler en marcha, el valor actual se **deduce** sumando los incrementos
    /// transcurridos desde `anchor`; para uno parado o en cascada, es el valor
    /// actual directo.
    counter: u16,
    /// Ciclo de referencia desde el que se cuenta el avance por prescaler.
    anchor: u64,
    /// `true` si el timer está activo (el bit enable está puesto). Se guarda aparte
    /// del control para detectar el **flanco** 0→1 que lo arranca.
    running: bool,
    /// Ciclo del desborde **programado** en el scheduler, o `None` si no hay
    /// ninguno (parado, o en cascada). Sirve para descartar eventos **obsoletos**:
    /// si un evento que vence no coincide con este ciclo, es de una programación
    /// anterior (el timer se reconfiguró) y se ignora.
    overflow_at: Option<u64>,
}

impl Timer {
    fn new() -> Self {
        Timer {
            reload: 0,
            control: 0,
            counter: 0,
            anchor: 0,
            running: false,
            overflow_at: None,
        }
    }

    /// El divisor del prescaler configurado (1/64/256/1024 ciclos por incremento).
    fn prescaler(&self) -> u64 {
        PRESCALER_DIV[(self.control & CTRL_PRESCALER) as usize]
    }
}

/// Los cuatro temporizadores de hardware. Vive dentro del [`crate::Bus`].
pub struct Timers {
    timers: [Timer; NUM_TIMERS],
}

impl Timers {
    /// Crea los cuatro timers en reposo (parados, contadores y recargas a cero).
    pub fn new() -> Self {
        Timers {
            timers: [Timer::new(), Timer::new(), Timer::new(), Timer::new()],
        }
    }

    /// `true` si el offset de I/O `io_off` cae en un registro de timer. Lo usa el
    /// bus para enrutar aquí el acceso.
    pub fn handles(io_off: u32) -> bool {
        (TIMER_IO_BASE..TIMER_IO_END).contains(&io_off)
    }

    /// Lee un byte de un registro de timer. `now` es el ciclo actual (para deducir
    /// el contador de un timer en marcha). El byte alto de `TMxCNT_H` (bits 8-15,
    /// no usados) lee 0.
    pub fn read_u8(&self, io_off: u32, now: u64) -> u8 {
        let (i, within) = index_and_offset(io_off);
        match within {
            0 => self.current_counter(i, now) as u8,
            1 => (self.current_counter(i, now) >> 8) as u8,
            2 => self.timers[i].control as u8,
            _ => (self.timers[i].control >> 8) as u8,
        }
    }

    /// Escribe un byte en un registro de timer. `now` es el ciclo actual y
    /// `scheduler` la cola donde (re)programar el desborde. Escribir el byte bajo
    /// del control (que lleva el bit enable) arranca, para o reconfigura el timer.
    pub fn write_u8(&mut self, io_off: u32, value: u8, now: u64, scheduler: &mut Scheduler<Event>) {
        let (i, within) = index_and_offset(io_off);
        match within {
            // TMxCNT_L (recarga): no toca el contador en marcha, solo el valor con
            // que se recargará al desbordar.
            0 => self.timers[i].reload = (self.timers[i].reload & 0xFF00) | u16::from(value),
            1 => self.timers[i].reload = (self.timers[i].reload & 0x00FF) | (u16::from(value) << 8),
            // TMxCNT_H byte bajo: aquí están todos los bits de control útiles
            // (prescaler, cascada, IRQ, enable).
            2 => self.write_control_low(i, value, now, scheduler),
            // Byte alto del control: no usado.
            _ => {}
        }
    }

    /// Procesa un **desborde** de timer que el scheduler ha disparado (lo llama
    /// [`crate::Bus::sync_to_cycle`]). `at` es el ciclo en que estaba programado.
    ///
    /// Descarta el evento si es **obsoleto** (el timer se reconfiguró tras
    /// programarlo): solo actúa si `at` coincide con el desborde que el timer
    /// espera.
    pub fn on_overflow(
        &mut self,
        i: usize,
        at: u64,
        scheduler: &mut Scheduler<Event>,
        irq: &mut InterruptControl,
    ) {
        if self.timers.get(i).and_then(|t| t.overflow_at) == Some(at) {
            self.do_overflow(i, at, scheduler, irq);
        }
    }

    /// `true` si **algún** timer en marcha podría despertar a la CPU de un `Halt`:
    /// tiene la IRQ activada (bit 6) y su fuente está habilitada en `IE`. Sin esto,
    /// el bucle no sabría si saltar el tiempo muerto del `Halt` o pararse
    /// (Mini-Hito 2.3e; ver [`crate::Bus::next_wakeup_cycle`]).
    pub fn can_wake(&self, irq: &InterruptControl) -> bool {
        (0..NUM_TIMERS).any(|i| {
            self.timers[i].running
                && self.timers[i].control & CTRL_IRQ != 0
                && irq.is_enabled(Interrupt::timer(i))
        })
    }

    // ---- Internos -------------------------------------------------------

    /// `true` si el timer `i` está en modo **cascada** (count-up). TM0 nunca lo
    /// está (no hay timer anterior que lo alimente).
    fn is_cascade(&self, i: usize) -> bool {
        i > 0 && self.timers[i].control & CTRL_CASCADE != 0
    }

    /// El valor **actual** del contador del timer `i` en el ciclo `now`. Para un
    /// timer por prescaler en marcha se deduce del tiempo transcurrido; parado o en
    /// cascada, es el valor almacenado.
    fn current_counter(&self, i: usize, now: u64) -> u16 {
        let t = &self.timers[i];
        if !t.running || self.is_cascade(i) {
            t.counter
        } else {
            let elapsed = now.saturating_sub(t.anchor) / t.prescaler();
            t.counter.wrapping_add(elapsed as u16)
        }
    }

    /// Aplica una escritura al byte bajo de `TMxCNT_H` y reacciona al cambio del
    /// bit enable (flanco de arranque/parada) o de la configuración.
    fn write_control_low(&mut self, i: usize, value: u8, now: u64, scheduler: &mut Scheduler<Event>) {
        let was_enabled = self.timers[i].running;
        self.timers[i].control = (self.timers[i].control & 0xFF00) | u16::from(value);
        let now_enabled = self.timers[i].control & CTRL_ENABLE != 0;

        if now_enabled && !was_enabled {
            // Flanco 0→1: el contador se carga con la recarga y el timer arranca.
            self.start(i, now, scheduler);
        } else if !now_enabled && was_enabled {
            // Flanco 1→0: se congela el contador en su valor actual.
            self.stop(i, now);
        } else if now_enabled {
            // Sigue activo pero pudo cambiar el prescaler/cascada: reprograma desde
            // el valor actual del contador.
            self.timers[i].counter = self.current_counter(i, now);
            self.timers[i].anchor = now;
            self.arm_overflow(i, now, scheduler);
        }
    }

    /// Arranca el timer `i`: carga el contador con la recarga y programa el desborde
    /// (si no es cascada).
    fn start(&mut self, i: usize, now: u64, scheduler: &mut Scheduler<Event>) {
        self.timers[i].counter = self.timers[i].reload;
        self.timers[i].anchor = now;
        self.timers[i].running = true;
        self.arm_overflow(i, now, scheduler);
    }

    /// Para el timer `i`: congela el contador y olvida el desborde programado (su
    /// evento, ya en la cola, quedará obsoleto y se ignorará al vencer).
    fn stop(&mut self, i: usize, now: u64) {
        self.timers[i].counter = self.current_counter(i, now);
        self.timers[i].running = false;
        self.timers[i].overflow_at = None;
    }

    /// Programa en el scheduler el próximo desborde del timer `i` (si avanza por
    /// prescaler). Un timer en cascada no se programa por tiempo: lo mueve el
    /// desborde del anterior, así que solo se anula su desborde pendiente.
    fn arm_overflow(&mut self, i: usize, now: u64, scheduler: &mut Scheduler<Event>) {
        if self.is_cascade(i) {
            self.timers[i].overflow_at = None;
            return;
        }
        let ticks_to_overflow = 0x1_0000 - u64::from(self.current_counter(i, now));
        let at = now + ticks_to_overflow * self.timers[i].prescaler();
        self.timers[i].overflow_at = Some(at);
        scheduler.schedule_at(at, Event::TimerOverflow { timer: i, at });
    }

    /// Aplica un desborde del timer `i` en el ciclo `now`: recarga, IRQ si procede,
    /// reprograma el siguiente (si va por prescaler) y propaga a la cascada.
    fn do_overflow(
        &mut self,
        i: usize,
        now: u64,
        scheduler: &mut Scheduler<Event>,
        irq: &mut InterruptControl,
    ) {
        self.timers[i].counter = self.timers[i].reload;
        self.timers[i].anchor = now;

        if self.timers[i].control & CTRL_IRQ != 0 {
            irq.request(Interrupt::timer(i));
        }

        // Un timer por prescaler reprograma su siguiente desborde; uno en cascada
        // espera de nuevo al timer anterior.
        if self.is_cascade(i) {
            self.timers[i].overflow_at = None;
        } else {
            self.arm_overflow(i, now, scheduler);
        }

        // Cascada: si el timer siguiente está en count-up y activo, este desborde
        // lo incrementa (y puede encadenar otro desborde).
        let next = i + 1;
        if next < NUM_TIMERS && self.timers[next].running && self.is_cascade(next) {
            self.cascade_increment(next, now, scheduler, irq);
        }
    }

    /// Incrementa en uno el contador del timer en cascada `i`; si desborda, aplica
    /// su propio desborde (recarga, IRQ y la siguiente cascada).
    fn cascade_increment(
        &mut self,
        i: usize,
        now: u64,
        scheduler: &mut Scheduler<Event>,
        irq: &mut InterruptControl,
    ) {
        let (next, overflowed) = self.timers[i].counter.overflowing_add(1);
        self.timers[i].counter = next;
        if overflowed {
            self.do_overflow(i, now, scheduler, irq);
        }
    }
}

impl Default for Timers {
    fn default() -> Self {
        Self::new()
    }
}

/// Descompone un offset de I/O de timer en `(índice de timer, byte dentro del
/// timer)`: byte 0/1 = `CNT_L`, 2/3 = `CNT_H`.
fn index_and_offset(io_off: u32) -> (usize, u32) {
    let local = io_off - TIMER_IO_BASE;
    ((local / TIMER_STRIDE) as usize, local % TIMER_STRIDE)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Offset de I/O de `TMxCNT_L`/`TMxCNT_H` del timer `i`.
    fn cnt_l(i: u32) -> u32 {
        TIMER_IO_BASE + i * TIMER_STRIDE
    }
    fn cnt_h(i: u32) -> u32 {
        cnt_l(i) + 2
    }

    /// Escribe `TMxCNT_L` (recarga, 16 bits) del timer `i`.
    fn set_reload(t: &mut Timers, sched: &mut Scheduler<Event>, i: u32, reload: u16) {
        t.write_u8(cnt_l(i), reload as u8, 0, sched);
        t.write_u8(cnt_l(i) + 1, (reload >> 8) as u8, 0, sched);
    }

    /// Escribe `TMxCNT_H` (control) del timer `i` en el ciclo `now`.
    fn set_control(t: &mut Timers, sched: &mut Scheduler<Event>, i: u32, ctrl: u16, now: u64) {
        t.write_u8(cnt_h(i), ctrl as u8, now, sched);
        t.write_u8(cnt_h(i) + 1, (ctrl >> 8) as u8, now, sched);
    }

    #[test]
    fn handles_reconoce_el_bloque_de_timers() {
        assert!(!Timers::handles(0x0FF));
        assert!(Timers::handles(0x100)); // TM0CNT_L
        assert!(Timers::handles(0x10F)); // TM3CNT_H byte alto
        assert!(!Timers::handles(0x110));
    }

    #[test]
    fn el_contador_avanza_segun_el_prescaler() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        // TM0: recarga 0, prescaler ÷1, enable. Arranca en el ciclo 0.
        set_reload(&mut t, &mut sched, 0, 0);
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE, 0);
        // Tras 100 ciclos, el contador vale 100 (÷1).
        assert_eq!(t.read_u8(cnt_l(0), 100), 100);
        assert_eq!(t.read_u8(cnt_l(0) + 1, 100), 0);

        // Con prescaler ÷256, el contador avanza 1 cada 256 ciclos.
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE | 0b10, 0); // prescaler índice 2 = ÷256
        assert_eq!(t.read_u8(cnt_l(0), 256), 1);
        assert_eq!(t.read_u8(cnt_l(0), 1024), 4);
    }

    #[test]
    fn programa_el_desborde_en_el_ciclo_esperado() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        // Recarga 0xFFF0 → faltan 0x10 = 16 incrementos para desbordar. Prescaler
        // ÷64 → 16 × 64 = 1024 ciclos.
        set_reload(&mut t, &mut sched, 0, 0xFFF0);
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE | 0b01, 0); // ÷64
        assert_eq!(sched.next_event_cycle(), Some(1024), "desborda en el ciclo 1024");
    }

    #[test]
    fn al_desbordar_recarga_y_lanza_la_irq() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        let mut irq = InterruptControl::new();
        irq.write_u8(0x200, 0xFF); // IE: habilita las primeras 8 fuentes (incl. Timer0)
        // Recarga 0xFFFE → 2 incrementos, ÷1 → desborda en el ciclo 2.
        set_reload(&mut t, &mut sched, 0, 0xFFFE);
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE | CTRL_IRQ, 0);
        assert_eq!(sched.next_event_cycle(), Some(2));

        // Disparar el evento del desborde.
        sched.advance_to(2);
        let event = sched.pop_due().expect("hay un desborde vencido");
        let Event::TimerOverflow { timer, at } = event;
        t.on_overflow(timer, at, &mut sched, &mut irq);

        assert!(irq.raised(), "el desborde levantó la IRQ del Timer0");
        // Tras recargar, vuelve a contar desde 0xFFFE y reprograma (ciclo 2+2=4).
        assert_eq!(t.read_u8(cnt_l(0), 2), 0xFE);
        assert_eq!(sched.next_event_cycle(), Some(4));
    }

    #[test]
    fn dos_timers_en_cascada_cuentan_los_desbordes_del_anterior() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        let mut irq = InterruptControl::new();
        // TM0: recarga 0xFFFF, ÷1 → desborda cada ciclo. TM1: cascada, recarga 0.
        set_reload(&mut t, &mut sched, 0, 0xFFFF);
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE, 0);
        set_reload(&mut t, &mut sched, 1, 0);
        set_control(&mut t, &mut sched, 1, CTRL_ENABLE | CTRL_CASCADE, 0);

        // Procesar 3 desbordes de TM0 (en los ciclos 1, 2, 3).
        for ciclo in 1..=3u64 {
            sched.advance_to(ciclo);
            while let Some(Event::TimerOverflow { timer, at }) = sched.pop_due() {
                t.on_overflow(timer, at, &mut sched, &mut irq);
            }
        }
        // TM1 ha contado los 3 desbordes de TM0.
        assert_eq!(t.read_u8(cnt_l(1), 3), 3, "TM1 cuenta los desbordes de TM0");
    }

    #[test]
    fn deshabilitar_congela_el_contador() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE, 0); // ÷1 desde 0
        // En el ciclo 50 lo deshabilitamos: el contador se congela en 50.
        set_control(&mut t, &mut sched, 0, 0, 50);
        assert_eq!(t.read_u8(cnt_l(0), 999), 50, "parado, el contador no avanza");
    }

    #[test]
    fn can_wake_distingue_un_timer_con_irq_habilitada() {
        let mut t = Timers::new();
        let mut sched = Scheduler::new();
        let mut irq = InterruptControl::new();
        // Timer con IRQ pero IE sin habilitar: no puede despertar.
        set_control(&mut t, &mut sched, 0, CTRL_ENABLE | CTRL_IRQ, 0);
        assert!(!t.can_wake(&irq));
        // Al habilitar el Timer0 en IE, ya puede.
        irq.write_u8(0x200, 0x08); // IE bit 3 = Timer0
        assert!(t.can_wake(&irq));
    }
}
