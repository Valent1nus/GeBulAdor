//! El **Scheduler**: la cola de eventos del hardware ordenada por ciclo
//! (Mini-Hito 2.2d).
//!
//! ## Por qué un scheduler y no un bucle que pregunta a cada ciclo
//!
//! Una GBA tiene varias piezas que "hacen algo" en un ciclo futuro concreto: la
//! PPU termina una línea (H-Blank) cada 1232 ciclos, un timer desborda dentro de
//! N ciclos, un DMA arranca al entrar en V-Blank... La forma ingenua de emular
//! esto sería, en **cada** ciclo, preguntar a cada componente "¿te toca ya?".
//! Eso es lentísimo (la mayoría de las veces la respuesta es "no") y propenso a
//! errores de temporización.
//!
//! El patrón estándar —y el que la Fase 4 NECESITA para un *Lockstep* fiable— es
//! al revés: cada componente **programa** en qué ciclo exacto quiere que pase
//! algo, y el scheduler mantiene todos esos avisos **ordenados por ciclo
//! objetivo**. El bucle principal avanza el reloj global y, cuando este alcanza
//! (o supera) el ciclo de un evento, lo **dispara**. Así solo se "visita" un
//! componente justo cuando le toca, no a cada ciclo.
//!
//! ## Diseño: el scheduler solo ORDENA y ENTREGA; el llamador MANEJA
//!
//! Este [`Scheduler`] deliberadamente **no sabe qué hacer** cuando un evento
//! vence: solo guarda etiquetas (de tipo `E`) asociadas a un ciclo y las devuelve
//! en orden. Es **genérico** sobre el tipo de evento `E` por dos razones:
//!
//! 1. **Aún no existen los eventos reales.** Los tipos concretos (H-Blank,
//!    overflow de un timer, fin de un DMA...) llegan con sus subsistemas en el
//!    Mini-Hito 2.3e (timers) y 2.4b (PPU). Hacer el scheduler genérico evita
//!    inventar hoy un `enum` lleno de variantes muertas, y deja la mecánica
//!    —que es la misma sea cual sea el evento— probada y lista de antemano.
//! 2. **Evita el infierno de los *callbacks* con referencias a la consola.** Si
//!    el scheduler guardara *closures* que tocan la GBA, chocaría con el *borrow
//!    checker* (el scheduler vivirá DENTRO de la GBA: no puede prestarse a sí
//!    mismo la GBA entera). En su lugar entrega el evento y es la GBA —que tiene
//!    acceso a todo su estado— quien lo maneja:
//!
//!    ```ignore
//!    sched.advance(ciclos_consumidos_por_la_cpu);
//!    while let Some(evento) = sched.pop_due() {
//!        match evento { /* la GBA actúa con su contexto completo */ }
//!    }
//!    ```
//!
//! ## Determinismo (no negociable para el Lockstep de la Fase 4)
//!
//! Dos emuladores que ejecuten la misma ROM deben disparar exactamente los
//! mismos eventos en el mismo orden, o aparecerá *desync*. Por eso el orden de
//! disparo es **total y determinista**: primero por ciclo objetivo y, a igualdad
//! de ciclo, por **orden de inserción** (FIFO) gracias a un número de secuencia.
//! El desempate nunca se deja "al azar" interno del montículo.
//!
//! ## Estado actual (2.2d): infraestructura, todavía sin integrar
//!
//! Esta pieza se monta ahora —igual que el bucle (2.2a), el oráculo de test
//! (2.2b) y el contador de ciclos (2.2c)— pero **aún no se enchufa** al bucle de
//! ejecución: todavía no hay eventos reales que disparar. Por eso el `Scheduler`
//! lleva de momento su **propio** reloj ([`Scheduler::now`]), que se avanza a
//! mano. Cuando existan los primeros eventos (timers, 2.3e), el reloj de la CPU
//! ([`crate::Cpu::cycles`]) y el del scheduler se unificarán, y el bucle drenará
//! los eventos vencidos tras cada instrucción.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Un evento ya colocado en la cola: el ciclo en que debe dispararse, un número
/// de secuencia para desempatar de forma determinista, y la etiqueta del evento.
///
/// Es un detalle interno del [`Scheduler`]; el llamador nunca lo ve (solo recibe
/// el `event` de vuelta en [`Scheduler::pop_due`]).
///
/// ## Por qué implementa `Ord` "a mano" y solo sobre `(when, seq)`
///
/// La cola de prioridad ([`BinaryHeap`]) necesita ordenar sus elementos, pero
/// queremos ordenarlos **solo por el ciclo** (y el `seq` como desempate), nunca
/// por el contenido del evento —que podría ni siquiera ser comparable—. Por eso
/// no podemos derivar `Ord` (eso exigiría `E: Ord`): lo implementamos
/// manualmente ignorando `event`, lo que mantiene el `Scheduler` genérico sobre
/// **cualquier** `E`.
struct ScheduledEvent<E> {
    /// Ciclo (del reloj global) en el que este evento debe dispararse.
    when: u64,
    /// Orden de inserción. A igualdad de `when`, vence antes el de `seq` menor
    /// (FIFO). Garantiza un orden total determinista (ver doc del módulo).
    seq: u64,
    /// La etiqueta del evento que se devolverá al llamador cuando venza.
    event: E,
}

impl<E> PartialEq for ScheduledEvent<E> {
    fn eq(&self, other: &Self) -> bool {
        // Dos entradas son "iguales" solo si coinciden ciclo y secuencia. Como
        // el `seq` es único por inserción, en la práctica esto nunca es `true`
        // para entradas distintas: el orden es estricto.
        self.when == other.when && self.seq == other.seq
    }
}

impl<E> Eq for ScheduledEvent<E> {}

impl<E> Ord for ScheduledEvent<E> {
    fn cmp(&self, other: &Self) -> Ordering {
        // `BinaryHeap` es un montículo de MÁXIMOS: `peek`/`pop` devuelven el
        // elemento "mayor". Nosotros queremos que el "mayor" sea el que antes
        // debe dispararse: menor `when` y, a igualdad, menor `seq`. Por eso
        // invertimos el orden natural (comparamos `other` contra `self`).
        other
            .when
            .cmp(&self.when)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

impl<E> PartialOrd for ScheduledEvent<E> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Cola de eventos del hardware ordenada por ciclo objetivo (Mini-Hito 2.2d).
///
/// Mantiene un **reloj global** ([`Scheduler::now`], en ciclos) y un montículo de
/// eventos pendientes. Se programa con [`Scheduler::schedule_in`] /
/// [`Scheduler::schedule_at`], se hace avanzar el reloj con
/// [`Scheduler::advance`], y se drenan los eventos ya vencidos con
/// [`Scheduler::pop_due`]. Ver el patrón de uso en la documentación del módulo.
///
/// `E` es el tipo de la etiqueta de evento (ver módulo: hoy genérico, mañana un
/// `enum` concreto de eventos de hardware).
pub struct Scheduler<E> {
    /// Eventos pendientes, como montículo ordenado por `(when, seq)`.
    heap: BinaryHeap<ScheduledEvent<E>>,
    /// Reloj global, en ciclos transcurridos.
    now: u64,
    /// Próximo número de secuencia a asignar (para el desempate FIFO).
    next_seq: u64,
}

impl<E> Scheduler<E> {
    /// Crea un scheduler vacío con el reloj a cero.
    pub fn new() -> Self {
        Scheduler {
            heap: BinaryHeap::new(),
            now: 0,
            next_seq: 0,
        }
    }

    /// El valor actual del reloj global, en ciclos.
    pub fn now(&self) -> u64 {
        self.now
    }

    /// `true` si no hay ningún evento pendiente.
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// Número de eventos pendientes en la cola.
    pub fn len(&self) -> usize {
        self.heap.len()
    }

    /// Programa `event` para dispararse en el ciclo **absoluto** `when`.
    ///
    /// Si `when` es menor o igual que el reloj actual ([`Scheduler::now`]), el
    /// evento se considera ya vencido y lo devolverá la próxima llamada a
    /// [`Scheduler::pop_due`].
    pub fn schedule_at(&mut self, when: u64, event: E) {
        let seq = self.next_seq;
        // El reloj de la GBA tardaría ~35 000 años en agotar un `u64`, así que
        // este `wrapping` es solo una red de seguridad: nunca llega a envolver.
        self.next_seq = self.next_seq.wrapping_add(1);
        self.heap.push(ScheduledEvent { when, seq, event });
    }

    /// Programa `event` para dispararse dentro de `delay` ciclos a partir de
    /// **ahora** ([`Scheduler::now`]). Es la forma relativa de
    /// [`Scheduler::schedule_at`] —la más habitual: "este timer desborda dentro
    /// de N ciclos"—. Un `delay` de 0 vence de inmediato.
    pub fn schedule_in(&mut self, delay: u64, event: E) {
        self.schedule_at(self.now.wrapping_add(delay), event);
    }

    /// El ciclo objetivo del próximo evento a vencer, o `None` si la cola está
    /// vacía. Útil para que el bucle principal sepa cuántos ciclos puede correr
    /// la CPU "de un tirón" sin saltarse ningún evento.
    pub fn next_event_cycle(&self) -> Option<u64> {
        self.heap.peek().map(|scheduled| scheduled.when)
    }

    /// Avanza el reloj global hasta el ciclo absoluto `cycle`.
    ///
    /// El reloj es **monotónico**: nunca retrocede. Pedirle un `cycle` anterior
    /// al actual delataría un bug del llamador; el `debug_assert!` lo caza en
    /// builds de depuración y en release se ignora (no se toca el reloj).
    pub fn advance_to(&mut self, cycle: u64) {
        debug_assert!(
            cycle >= self.now,
            "el reloj del scheduler no puede retroceder (now={}, cycle={cycle})",
            self.now
        );
        if cycle > self.now {
            self.now = cycle;
        }
    }

    /// Avanza el reloj global `delta` ciclos. Es la forma relativa de
    /// [`Scheduler::advance_to`]: la que usará el bucle tras cada instrucción
    /// ("la CPU ha consumido `delta` ciclos").
    pub fn advance(&mut self, delta: u64) {
        self.advance_to(self.now.wrapping_add(delta));
    }

    /// Saca y devuelve el próximo evento **ya vencido** (cuyo ciclo objetivo es
    /// `<= now`), o `None` si el siguiente evento aún es futuro (o no hay
    /// ninguno).
    ///
    /// Se llama en bucle para drenar todos los vencidos tras avanzar el reloj
    /// (ver el patrón en la documentación del módulo). El orden de salida es el
    /// determinista descrito allí: por ciclo y, a igualdad, por inserción.
    pub fn pop_due(&mut self) -> Option<E> {
        let when = self.next_event_cycle()?;
        if when <= self.now {
            // `pop` no puede devolver `None`: `next_event_cycle` (un `peek`)
            // acaba de confirmar que hay al menos un elemento.
            self.heap.pop().map(|scheduled| scheduled.event)
        } else {
            None
        }
    }
}

impl<E> Default for Scheduler<E> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn un_scheduler_nuevo_esta_vacio_y_a_cero() {
        let sched: Scheduler<&str> = Scheduler::new();
        assert_eq!(sched.now(), 0);
        assert!(sched.is_empty());
        assert_eq!(sched.len(), 0);
        assert_eq!(sched.next_event_cycle(), None);
    }

    #[test]
    fn evento_ficticio_se_dispara_exactamente_al_llegar_a_su_ciclo() {
        // La "Prueba" del Mini-Hito 2.2d: un evento ficticio a 100 ciclos se
        // ejecuta exactamente al alcanzar ese valor, ni antes ni después.
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.schedule_in(100, "ficticio");
        assert_eq!(sched.next_event_cycle(), Some(100));

        // Justo un ciclo antes: aún no vence.
        sched.advance(99);
        assert_eq!(sched.now(), 99);
        assert_eq!(sched.pop_due(), None, "a 99 ciclos todavía no toca");

        // Al alcanzar el ciclo 100 exacto: se dispara.
        sched.advance(1);
        assert_eq!(sched.now(), 100);
        assert_eq!(sched.pop_due(), Some("ficticio"), "a 100 ciclos se dispara");

        // Y solo una vez: la cola queda vacía.
        assert_eq!(sched.pop_due(), None);
        assert!(sched.is_empty());
    }

    #[test]
    fn nada_vence_antes_de_su_ciclo() {
        let mut sched: Scheduler<u32> = Scheduler::new();
        sched.schedule_in(50, 7);
        // Sin avanzar el reloj, nada está vencido.
        assert_eq!(sched.pop_due(), None);
        assert_eq!(sched.len(), 1);
    }

    #[test]
    fn los_eventos_salen_en_orden_de_ciclo_creciente() {
        // Se programan desordenados; deben salir ordenados por ciclo.
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.schedule_at(300, "tres");
        sched.schedule_at(100, "uno");
        sched.schedule_at(200, "dos");

        // El próximo a vencer es siempre el de menor ciclo.
        assert_eq!(sched.next_event_cycle(), Some(100));

        sched.advance_to(1000); // pasa de todos
        assert_eq!(sched.pop_due(), Some("uno"));
        assert_eq!(sched.pop_due(), Some("dos"));
        assert_eq!(sched.pop_due(), Some("tres"));
        assert_eq!(sched.pop_due(), None);
    }

    #[test]
    fn desempate_fifo_a_igual_ciclo() {
        // Tres eventos en el MISMO ciclo: deben salir en orden de inserción.
        // Intercalamos otro ciclo en medio para asegurarnos de que el desempate
        // no es un accidente del montículo.
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.schedule_at(100, "a");
        sched.schedule_at(100, "b");
        sched.schedule_at(50, "previo");
        sched.schedule_at(100, "c");

        sched.advance_to(100);
        assert_eq!(sched.pop_due(), Some("previo"), "primero el de ciclo menor");
        assert_eq!(sched.pop_due(), Some("a"), "luego, FIFO: a, b, c");
        assert_eq!(sched.pop_due(), Some("b"));
        assert_eq!(sched.pop_due(), Some("c"));
        assert_eq!(sched.pop_due(), None);
    }

    #[test]
    fn schedule_in_es_relativo_al_now_actual() {
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.advance(40);
        // "dentro de 10 ciclos" desde now=40 → ciclo absoluto 50.
        sched.schedule_in(10, "relativo");
        assert_eq!(sched.next_event_cycle(), Some(50));

        sched.advance(9); // now = 49
        assert_eq!(sched.pop_due(), None);
        sched.advance(1); // now = 50
        assert_eq!(sched.pop_due(), Some("relativo"));
    }

    #[test]
    fn un_evento_en_el_pasado_vence_de_inmediato() {
        // Programar para un ciclo ya superado lo deja listo para el próximo drenaje.
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.advance(100);
        sched.schedule_at(30, "tarde");
        assert_eq!(sched.pop_due(), Some("tarde"));
    }

    #[test]
    fn schedule_in_cero_vence_en_el_ciclo_actual() {
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.advance(10);
        sched.schedule_in(0, "ya");
        assert_eq!(sched.pop_due(), Some("ya"));
    }

    #[test]
    fn pop_due_drena_solo_los_vencidos_y_conserva_los_futuros() {
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.schedule_at(10, "pronto");
        sched.schedule_at(20, "medio");
        sched.schedule_at(100, "lejos");

        // Avanzamos hasta 50: vencen los dos primeros, no el tercero.
        sched.advance_to(50);
        assert_eq!(sched.pop_due(), Some("pronto"));
        assert_eq!(sched.pop_due(), Some("medio"));
        assert_eq!(sched.pop_due(), None, "el de ciclo 100 aún es futuro");
        assert_eq!(sched.len(), 1);
        assert_eq!(sched.next_event_cycle(), Some(100));

        // Y al llegar a 100, sale el último.
        sched.advance_to(100);
        assert_eq!(sched.pop_due(), Some("lejos"));
        assert!(sched.is_empty());
    }

    #[test]
    fn un_evento_puede_reprogramar_otro_el_patron_de_los_timers() {
        // Simula lo que hará un timer periódico: cada vez que vence, se vuelve a
        // programar para dentro de su periodo. Comprobamos que el scheduler
        // soporta encadenar disparos sin perder la cadencia.
        let mut sched: Scheduler<&str> = Scheduler::new();
        const PERIODO: u64 = 1232; // ciclos de una línea de la PPU, como guiño
        sched.schedule_in(PERIODO, "tick");

        let mut disparos = 0;
        // Emulamos 5 periodos avanzando el reloj en saltos.
        for _ in 0..5 {
            sched.advance(PERIODO);
            while let Some(evento) = sched.pop_due() {
                assert_eq!(evento, "tick");
                disparos += 1;
                // El manejador se reprograma a sí mismo (como un timer real).
                sched.schedule_in(PERIODO, "tick");
            }
        }
        assert_eq!(disparos, 5);
        // Tras 5 periodos, el reloj va por 5×PERIODO y queda un "tick" futuro.
        assert_eq!(sched.now(), 5 * PERIODO);
        assert_eq!(sched.next_event_cycle(), Some(6 * PERIODO));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "no puede retroceder")]
    fn el_reloj_no_retrocede_en_debug() {
        // El reloj es monotónico: pedirle ir hacia atrás es un bug del llamador,
        // y en builds de depuración lo delata el `debug_assert!`.
        let mut sched: Scheduler<&str> = Scheduler::new();
        sched.advance(100);
        sched.advance_to(50);
    }
}
