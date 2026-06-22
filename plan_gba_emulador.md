# Plan de Desarrollo: Emulador de Game Boy Advance en Rust

> Arquitectura desacoplada `core` / `frontend` desde el día 1, para permitir portar a móvil sin reescribir el núcleo.

## ⏱️ Expectativas realistas antes de empezar

Este es un proyecto de **varios meses**, no de unas semanas — y eso es normal, no un fallo de planificación. Como aprendes Rust y emulación de hardware a la vez, cuenta con que cada fase tomará más tiempo del que "debería" tomar a alguien con experiencia previa. Una estimación honesta, dedicando un puñado de horas a la semana:

| Fase | Contenido | Tiempo estimado (principiante) | Dónde suele abandonarse el proyecto |
|---|---|---|---|
| 1 | Workspace, ventana, parseo de cabecera | 1–2 semanas | Rara vez aquí — es la parte más gratificante |
| 2.1 | CPU básica (fetch/decode/execute, pipeline) | 3–5 semanas | **Punto de abandono #1**: el decode condicional + pipeline rompe la intuición inicial |
| 2.2 | Ciclos, scheduler, primeras ROMs de test | 2–4 semanas | Frustración por errores de test sin contexto suficiente para depurarlos |
| 2.3 | BIOS, DMA, IRQ, SIO | 3–6 semanas | **Punto de abandono #2**: cantidad de registros de hardware sin gratificación visual |
| 2.4 | PPU (gráficos) | 4–8 semanas | Es la fase más larga, pero la que da más motivación al ver imágenes reales |
| 3 | Input y guardado | 1–2 semanas | Rara vez — es la fase más sencilla de todo el proyecto |
| 4 | Cable Link online | 3–6 semanas | Requiere que TODO lo anterior sea ciclo-preciso; si la Fase 2 quedó floja, aquí se paga |

**Total realista: entre 4 y 8 meses** a tiempo parcial, asumiendo que no abandonas en los puntos críticos señalados. Esto no es pesimismo — es el patrón que se repite en casi todos los proyectos de emulación de hobbyistas que llegan a buen puerto. Saber dónde vas a querer tirar la toalla es parte de superarlo.

> **Regla de oro para principiante en Rust:** si en cualquier mini-hito llevas más de ~3-4 sesiones de trabajo sin avanzar y sientes que el problema es el lenguaje (lifetimes, ownership, tipos) y no la lógica del emulador, **para y dedica un día entero solo a Rust** (el libro oficial "The Rust Book", capítulos de ownership y enums, son los que más se usan aquí). Mezclar "estoy aprendiendo Rust" con "esto no compila por una razón de hardware" es la combinación que más desmotiva.

---

## 🏗️ Arquitectura general (decisión previa a la Fase 1)

Antes de tocar código, se monta un **Cargo Workspace** con dos crates:

```
[GBA Workspace]
 ├── gba_core/          (librería pura, CERO dependencias gráficas)
 │    ├── cpu.rs         (registros, fetch/decode/execute)
 │    ├── bus.rs          (mapa de memoria)
 │    ├── ppu.rs          (genera un framebuffer crudo RGBA)
 │    └── scheduler.rs    (cola de eventos por ciclos)
 └── gba_desktop/       (binario ejecutable, usa minifb o pixels)
      └── main.rs        (ventana, teclado, le pasa el framebuffer al core)
```

`gba_core` nunca importa una librería de ventanas. Solo expone un array de bytes `240×160×4` (RGBA) que el frontend pinta. Esto es lo que permite, más adelante, sustituir `gba_desktop` por una capa Android/iOS/WASM sin tocar una sola línea del core.

---

## 🛡️ Principios de seguridad transversales

Antes de empezar, conviene aclarar **qué tipo de riesgo es real** en un emulador, porque no es el que mucha gente imagina:

- **Un archivo `.gba` no es un ejecutable nativo.** No puede "contener un virus" que se ejecute directamente en tu sistema operativo — es un blob de datos que tu propia CPU emulada interpreta. El riesgo real es que una ROM corrupta o maliciosamente construida explote un bug en **tu código Rust** (un índice fuera de rango, una suposición de tamaño que no se cumple) y provoque un crash, un comportamiento indefinido, o en el peor caso una corrupción de memoria si usas `unsafe`.
- **El Cable Link online sí es una superficie de ataque real**, porque ahí aceptas paquetes de un proceso remoto que no controlas y los conviertes en estado de tu simulación. Es la parte del proyecto que más justifica tratamiento explícito de seguridad.
- **Rust ya te protege de gran parte de esto por diseño**: el compilador rechaza por defecto los accesos a memoria fuera de límites, los `Vec` y slices hacen comprobación de límites en runtime (panican en vez de corromper memoria), y no hay punteros crudos salvo que tú mismo escribas `unsafe`. Esto no es magia, pero significa que la disciplina de "validar antes de confiar" en otros lenguajes aquí se traduce sobre todo en **decidir conscientemente cuándo evitar `unwrap()` y panics no controlados**, no en perseguir buffer overflows clásicos de C/C++.

Reglas generales que se aplican en todos los hitos relevantes (marcadas explícitamente más abajo donde corresponde):

1. **Nunca usar `unwrap()` o indexado directo (`array[i]`) sobre datos que vienen de un archivo o de la red.** Usa `get()` que devuelve `Option`, y maneja el caso `None` con un error explícito en vez de dejar que panique.
2. **Validar tamaños antes de leer, no después.** Si vas a leer 4 bytes en el offset `X`, comprueba primero que `X + 4 <= buffer.len()`.
3. **Tratar toda entrada de red como hostil por defecto**, incluso viniendo de "tu propio emulador" en la otra punta — si mañana alguien hace un cliente alternativo, o hay un MITM, tu lado del protocolo debe sobrevivir a basura.
4. **Aislar el `unsafe`, si llega a aparecer** (por ejemplo, para optimizar el bus de memoria más adelante): que esté concentrado en pocas funciones pequeñas y bien comentadas, nunca disperso por el código.

---

## 📁 Fase 1: Investigación y Preparación

### ✅ Mini-Hito 1.1a — Hola Ventana
**Objetivo:** Crear el workspace y abrir la ventana gráfica.
**Tarea:** `cargo new --lib gba_core` y `cargo new gba_desktop`. Añade `minifb` o `pixels` solo en `gba_desktop`. Bucle principal en `main.rs` que abra una ventana de 240×160 píxeles pintada de un color sólido.
**Prueba:** `cargo run` muestra la ventana sin cerrarse ni congelar el PC.

> 💡 **Para quien viene de otros lenguajes:** en Rust vas a usar tipos enteros explícitos todo el rato (`u8`, `u16`, `u32`, `i32`...). Acostúmbrate desde ya a pensar "¿este valor son 8, 16 o 32 bits, con o sin signo?" antes de declarar una variable — en un emulador esto no es un detalle estético, es la diferencia entre un registro correcto y un bug de desbordamiento silencioso.

### ✅ Mini-Hito 1.2a — Cargar bytes en memoria
**Objetivo:** Leer un archivo externo en un array de Rust.
**Tarea:** `std::fs::File` + `std::io::Read` para abrir una ROM `.gba` de prueba legal. Cárgala en un `Vec<u8>`. Imprime el tamaño total en bytes.
**Prueba:** `Archivo cargado con éxito. Tamaño: 16777216 bytes (16MB).`

> 🛡️ **Seguridad — validación de tamaño desde el primer hito:** una GBA real tiene un máximo de 32MB de espacio de cartucho direccionable. Antes de aceptar el archivo, comprueba que su tamaño no excede ese límite (y, razonablemente, que no es sospechosamente pequeño — por debajo de unos pocos cientos de bytes ni siquiera cabe una cabecera válida). Esto evita que un archivo gigante o vacío provoque asignaciones de memoria descontroladas o panics más adelante al intentar leer la cabecera.

### ✅ Mini-Hito 1.2b — Parsear la cabecera
**Objetivo:** Extraer texto legible del binario.
**Tarea:** Los bytes `0xA0`–`0xAB` son el título (ASCII); conviértelos a `String`. Los bytes `0xAC`–`0xAF` son el código del juego.
**Prueba:** La terminal imprime el nombre exacto del juego (ej. `SUPER MARIOA`).

> 🛡️ **Seguridad — esto es exactamente donde aplica la Regla 1 y 2 de arriba:** este es el primer punto del proyecto donde indexas un `Vec<u8>` con offsets fijos basados en una especificación, asumiendo que el archivo los tiene. Un archivo `.gba` corrupto o trivialmente más pequeño que `0xAB` bytes haría panicar tu programa con indexado directo (`rom[0xA0..0xAB]` sobre un slice más corto entra en pánico). Usa `rom.get(0xA0..0xAC)` (que devuelve `Option<&[u8]>`) y devuelve un error legible si el archivo es demasiado pequeño, en vez de dejar que el programa crashee. Además, los bytes del título no están garantizados a ser ASCII válido en un archivo malformado — usa `String::from_utf8_lossy` en vez de asumir UTF-8/ASCII estricto, para no añadir un segundo punto de fallo.

---

## 🧠 Fase 2: El Núcleo (ARM7TDMI)

El ARM7TDMI corre a 16.78 MHz y soporta dos sets de instrucciones: **ARM** (32 bits) y **THUMB** (16 bits).

### ✅ Mini-Hito 2.1a — Bus de memoria y registros
**Objetivo:** Crear el esqueleto donde vive la CPU.
**Tarea:**
1. En `cpu.rs`: estructura `Cpu` con array de 16 registros de 32 bits (`r0`–`r15`) más el registro de estado `CPSR`.
2. En `bus.rs`: estructura `Bus` con el `Vec<u8>` de la ROM y el mapa de memoria (`0x08000000` → ROM, `0x06000000` → VRAM, etc.).

**Prueba:** Que compile perfectamente.

> ⚠️ **Trampa que aparece más adelante pero conviene anticipar en el diseño:** el ARM7TDMI tiene **modos de CPU** (User, IRQ, Supervisor, FIQ...) y algunos registros tienen **copias separadas por modo** (banked registers) — en particular `r13` (SP) y `r14` (LR) tienen una copia distinta para cada modo, y el modo IRQ además banca `SPSR`. No necesitas implementarlo todavía, pero si diseñas `Cpu` con un único array plano de 16 registros sin sitio para estas copias, el Mini-Hito 2.3c (IRQ) te va a obligar a rehacer la estructura entera. Considera desde ahora un diseño donde los registros banked vivan en un array separado indexado por modo actual.

> ⚠️ **Segunda trampa de diseño, en el Bus:** la GBA hace **rotación de bytes en accesos desalineados** en vez de fallar o redondear de forma simple. Una lectura de 32 bits (`LDR`) desde una dirección no múltiplo de 4 no lanza un error en el hardware real: rota el word leído. Si tu función de lectura del Bus simplemente ignora los bits bajos de la dirección sin más, te va a "funcionar por accidente" en pruebas simples y fallar de forma muy confusa cuando una ROM real haga un acceso desalineado intencionadamente (cosa que ocurre).

> 🛡️ **Seguridad — el Bus es tu punto único de validación, trátalo como tal:** toda lectura/escritura de la CPU pasa por aquí, así que es el lugar natural para centralizar la Regla 2 ("validar tamaños antes de leer"). Diseña `Bus::read_u8/u16/u32` y `Bus::write_*` para que devuelvan un valor seguro (ej. `0` o `0xFF`, que es lo que suelen devolver consolas reales en lecturas de regiones no mapeadas) en vez de panicar, cuando la CPU pida una dirección fuera de cualquier región conocida. Una ROM corrupta o un bug en tu propio decode pueden generar direcciones arbitrarias (ej. tras saltar a un puntero mal calculado); el Bus es la última línea de defensa antes de que eso se convierta en un panic que cierra el emulador en mitad de una partida.


### ⬜ Mini-Hito 2.1b — El primer "Fetch"
**Objetivo:** Que la CPU lea su primera instrucción.

> ⚠️ **Corrección importante:** la GBA real arranca en `0x00000000` (BIOS), no en `0x08000000`. Para este hito puedes apuntar `PC` directamente a la ROM como atajo de desarrollo ("skip BIOS"), pero documéntalo como una decisión temporal — en el Mini-Hito 2.3a se corrige para arrancar desde la BIOS real, que es lo que necesitan muchos juegos para funcionar correctamente.

**Tarea:** Configura `r15` (PC) y escribe una función `fetch` que lea 4 bytes en Little-Endian desde la dirección apuntada.
**Prueba:** Imprimir la primera instrucción en hexadecimal (ej. `0xEA00002E`).

### ⬜ Mini-Hito 2.1c — Decodificar el modo ARM
**Objetivo:** Saber qué tipo de instrucción es, manejando correctamente las condiciones.

> ⚠️ **Detalle crítico que casi siempre se pasa por alto:** los bits 31-28 de toda instrucción ARM son un **código de condición** (Z, N, C, V del CPSR), no parte del opcode. El decode real funciona en dos pasos:
> 1. Extraer bits 31-28 y evaluar si la condición se cumple contra el CPSR actual.
> 2. Si no se cumple → la instrucción se descarta (actúa como NOP de 1 ciclo).
> 3. Si se cumple → recién entonces se decodifican los bits 27-20 para saber si es `ADD`, `SUB`, `MOV`, etc.
>
> Si no separas esto desde el principio, tu `match` se vuelve inmanejable en cuanto aparezcan instrucciones condicionales (`MOVEQ`, `BNE`, etc.), que son la mayoría del código real de los juegos.

**Tarea:** Función `decode` con el flujo de dos pasos descrito arriba, usando un `match` para el opcode. No se programa la lógica todavía, solo se identifica.
**Prueba:** Al pasarle `0xEA00002E`, la terminal imprime: `¡Es una instrucción de Salto (B / Branch)!`.

### ⬜ Mini-Hito 2.1c-bis — Decodificar el modo THUMB *(nuevo, antes ausente)*
**Objetivo:** Evitar la trampa de pensar que THUMB es "ARM comprimido".

> THUMB **no** es un subconjunto trivial de ARM con instrucciones más cortas — es un set de instrucciones de 16 bits con su propio formato de decode, sin código de condición embebido en la instrucción (las condicionales en THUMB son siempre saltos `B<cond>` independientes), con menos bits para inmediatos, y con acceso limitado a `r8`–`r15` en la mayoría de instrucciones de registro general. Tratar THUMB como "un caso particular de ARM" en el código suele acabar en un `match` con ramas mal mapeadas que parecen funcionar con instrucciones simples y fallan con las reales.

**Tarea:** Crea una función `decode_thumb` separada (no reutilices la de ARM), con su propia tabla de formatos de 16 bits.
**Prueba:** Pasarle una instrucción THUMB conocida (ej. `0x2005`, que es `MOV r0, #5` en THUMB) e identificarla correctamente como un formato distinto al de ARM.

### ⬜ Mini-Hito 2.1d — Tu primera ejecución
**Objetivo:** Que la CPU altere un registro por primera vez.
**Tarea:** Programa la lógica de una instrucción simple, como `MOV` o `ADD`.
**Prueba:** `MOV R0, #5` deja `R0 == 5`.

### ⬜ Mini-Hito 2.1e — El pipeline de 3 etapas *(nuevo, antes ausente)*
**Objetivo:** Modelar el desfase real del Program Counter.

> El procesador real no ejecuta Fetch→Decode→Execute de forma puramente secuencial para una sola instrucción: mientras ejecuta la instrucción N, ya está decodificando la N+1 y trayendo de memoria la N+2. Esto provoca que el valor de `PC` que "ve" una instrucción en ejecución esté adelantado:
> - **Modo ARM:** `PC = dirección_actual + 8`
> - **Modo THUMB:** `PC = dirección_actual + 4`
>
> Cualquier instrucción que lea `PC` para calcular una dirección (saltos relativos, `LDR PC, [...]`, etc.) asume este desfase. Si no se modela aquí, los primeros saltos calculados de cualquier ROM real fallarán de forma muy difícil de depurar más adelante.

**Tarea:** Ajusta la representación del `PC` para que el resto de la CPU vea siempre el valor con el offset de pipeline aplicado, separándolo del puntero "real" de fetch.
**Prueba:** Una instrucción que lea su propio `PC` obtiene el valor adelantado correcto, verificable contra un emulador de referencia (ej. mGBA en modo debug) o contra la documentación técnica del ARM7TDMI.

### ⬜ Mini-Hito 2.2a — El bucle de ejecución infinito
**Objetivo:** Que la CPU corra sola hasta atascarse.
**Tarea:** Mete Fetch→Decode→Execute en un bucle. Incrementa el PC en 4 (ARM) o 2 (THUMB) bytes tras cada instrucción que no sea un salto. `panic!` o `break` ante instrucciones no implementadas.
**Prueba:** El emulador procesa decenas de instrucciones en milisegundos hasta detenerse limpiamente en una no implementada.

### ⬜ Mini-Hito 2.2b — Primeras ROMs de test *(adelantado desde el plan original)*
**Objetivo:** Validar la CPU contra un oráculo externo en vez de a ciegas.

> Implementar las +100 instrucciones de ARM/THUMB sin validación continua es la forma más común de abandonar un proyecto de emulación. Conviene traer aquí — justo después de tener un bucle de ejecución mínimo — las ROMs de test como `arm.gba` (del repositorio `jsmolka/gba-tests`). Estas ROMs escriben resultados de test al puerto serie (`SIODATA`); puedes interceptar esas escrituras y redirigirlas a `println!` en tu terminal, viendo qué instrucción falla exactamente sin necesidad de tener gráficos funcionando.

**Tarea:** Cargar `arm.gba` (y más adelante `thumb.gba`), interceptar escrituras a SIODATA y volcarlas a consola.
**Prueba:** Las ROMs indican `PASS` o el número de test que falla, por consola.

### ⬜ Mini-Hito 2.2c — Contador de ciclos
**Objetivo:** Asociar tiempo real a cada instrucción.
**Tarea:** Contador total de ciclos ejecutados. Cada instrucción implementada devuelve cuántos ciclos consume (distinguiendo accesos **N** — no secuenciales, más lentos — de accesos **S** — secuenciales, más rápidos, según el tipo de memoria accedida).
**Prueba:** Ejecutar varias instrucciones y verificar que el contador aumenta correctamente y de forma consistente con la documentación de timings del hardware.

### ⬜ Mini-Hito 2.2d — Scheduler básico
**Objetivo:** Preparar la sincronización de hardware.
**Tarea:** Estructura `Scheduler` como cola de eventos ordenada por ciclo objetivo. Cuando el reloj global alcanza el ciclo de un evento, se dispara.
**Prueba:** Programar un evento ficticio a 100 ciclos y verificar que se ejecuta exactamente al llegar a ese valor.

*(Nota: este Scheduler es la pieza que después hará posible un Lockstep fiable en la Fase 4 — su precisión aquí no es opcional.)*

### ⬜ Mini-Hito 2.3a — Integrar la BIOS y el modo THUMB
**Objetivo:** Arrancar como lo hace el hardware real, cerrando el atajo del Hito 2.1b.
**Tarea:** Carga `gba_bios.bin` en el Bus en `0x0`. Haz que `PC` arranque realmente en `0x00000000`. Modifica `fetch` para leer 2 bytes cuando el flag de modo CPU indique THUMB.
**Prueba:** El bucle ejecuta las instrucciones reales de la BIOS de Nintendo desde el inicio, y eventualmente salta a la ROM del cartucho de forma natural (sin el atajo manual previo).

### ⬜ Mini-Hito 2.3b — DMA básico
**Objetivo:** Implementar transferencia automática de memoria.
**Tarea:** DMA0–DMA3 con copia inmediata.
**Prueba:** Copiar un bloque de memoria vía DMA y verificar el resultado.

### ⬜ Mini-Hito 2.3c — Sistema de IRQ
**Objetivo:** Permitir que los componentes notifiquen eventos a la CPU.
**Tarea:** Implementar `IE`, `IF`, `IME`.
**Prueba:** Generar una interrupción software y comprobar que la CPU salta al vector correspondiente.

### ⬜ Mini-Hito 2.3d — Registros SIO simulados
**Objetivo:** Familiarizarse con el hardware del Cable Link antes de necesitarlo en la Fase 4.
**Tarea:** Implementar `SIOCNT`, `RCNT`, `SIODATA` en el Bus, sin lógica real todavía.
**Prueba:** Leer y escribir valores desde la CPU y comprobar que se almacenan correctamente.

### ⬜ Mini-Hito 2.4a — Modo 3 Bitmap
**Objetivo:** Dibujar píxeles directamente desde VRAM.
**Tarea:** Implementar el modo gráfico 3 y mostrar el contenido de VRAM en el framebuffer del core.
**Prueba:** Ver una imagen generada manualmente.

### ⬜ Mini-Hito 2.4b — Renderizado por Scanlines
**Objetivo:** Emular la pantalla como hardware real, no como un framebuffer estático.

> La PPU dibuja línea a línea, no el frame completo de golpe. Tras 240 píxeles de una línea visible viene el **H-Blank**; tras las 160 líneas visibles vienen 68 líneas invisibles de **V-Blank** (líneas 160–227), que los juegos aprovechan para hacer cálculos pesados sin romper la imagen. Muchos efectos (agua, transparencias graduales) cambian registros gráficos en pleno H-Blank, línea por línea — si renderizas el frame completo al final, esos efectos simplemente no aparecerán.

**Tarea:** Dibujar línea a línea respetando el timing de H-Blank/V-Blank, disparando los eventos correspondientes en el Scheduler.
**Prueba:** Mostrar correctamente una imagen completa, y verificar que un efecto dependiente de H-Blank se renderiza bien.

### ⬜ Mini-Hito 2.4c — Tiles y Backgrounds
**Objetivo:** Implementar los modos 0, 1 y 2.
**Tarea:** Leer mapas de tiles y renderizar fondos.
**Prueba:** Mostrar correctamente menús y escenarios sencillos.

### ⬜ Mini-Hito 2.4d — Sprites (OAM)
**Objetivo:** Mostrar objetos móviles.
**Tarea:** Implementar lectura de OAM y renderizado de sprites.
**Prueba:** Ver personajes en movimiento.

---

## 🎮 Fase 3: Inputs, Guardado y Frontend

### ⬜ Hito 3.1 — Control y Mapeo del Teclado
**Acción:** Capturar eventos de teclado vía `minifb`/`sdl2` en `gba_desktop`. Mapear al registro `KEYINPUT` (`0x04000130`) en el core.
**Criterio de éxito:** Presionar "Start" saca al juego de la pantalla de título con control fluido.

> ⚠️ **Trampa clásica:** `KEYINPUT` está **invertido** — un bit a `0` significa "tecla pulsada" y a `1` significa "tecla suelta" (es lógica activa-baja, herencia directa del hardware). El valor de reposo (nada pulsado) es `0xFFFF` (los 10 bits usados en 1), no `0x0000`. Si lo inicializas a cero, el juego interpretará que están pulsadas todas las teclas a la vez desde el primer frame.

### ⬜ Hito 3.2 — Persistencia de Partidas (Save Files)
**Acción:** Detectar escrituras a las regiones de guardado del cartucho (SRAM/Flash/EEPROM) y volcarlas a un `.sav` local con `std::fs`.
**Criterio de éxito:** Guardar desde el menú del juego, cerrar el emulador, reabrirlo, y poder continuar la partida.

---

## 🌐 Fase 4: El Cable Link Online (Multijugador)

El Cable Link es un protocolo **síncrono por hardware**: la consola maestra envía un pulso de reloj y ambas consolas intercambian datos simultáneamente. Los juegos comerciales no toleran lag porque nunca lo necesitaron — asumen un cable de cobre de un metro.

> 🛡️ **Seguridad — esta es la fase donde el riesgo es real, no teórico.** A partir de aquí tu programa acepta bytes que vienen de un proceso que no controlas, ejecutándose en otro PC. Esos bytes terminan convirtiéndose en estado de tu simulación de hardware. El principio rector de toda la fase: **todo lo que llega por el socket es hostil hasta que se demuestre lo contrario**, incluso si "en teoría" solo te vas a conectar con amigos — basta un cliente modificado, un MITM en una red no confiable, o simplemente un bug en el otro extremo, para que lleguen datos que no esperas.

### ⬜ Hito 4.1 — Conexión Local Inter-proceso
**Acción:** Dos instancias del emulador en el mismo PC, comunicadas por Sockets UDP en `127.0.0.1`, transfiriendo los bytes de `SIODATA`.
**Criterio de éxito:** La Instancia A envía un byte de sincronización y la Instancia B lo recibe e interpreta inmediatamente.

> 🛡️ **Seguridad — valida la forma del paquete antes de tocarlo:** aunque sea localhost y "solo tú" en este hito, acostúmbrate ya a comprobar la longitud exacta del paquete recibido antes de indexarlo. Un paquete UDP truncado o más largo de lo esperado no debe poder causar un panic por indexado fuera de rango — descártalo silenciosamente (o con un log de depuración) si no tiene el tamaño exacto que tu protocolo espera.

### ⬜ Hito 4.2 — Cliente-Servidor básico
**Objetivo:** Conectar dos emuladores por Internet.
**Acción:** Servidor relay TCP que retransmite los datos SIO entre dos clientes.
**Criterio de éxito:** Dos usuarios en Internet intercambian paquetes SIO de forma estable.

> 🛡️ **Seguridad — el salto a Internet cambia el modelo de amenaza:** en localhost confiabas implícitamente en el otro proceso porque eras tú quien lo arrancaba. En Internet, el relay TCP habla con clientes arbitrarios. Como mínimo en este hito: (1) limita el tamaño máximo de mensaje que el servidor acepta de un cliente antes de reenviarlo — sin límite, un cliente malicioso o con un bug puede enviar paquetes gigantes y agotar memoria del servidor; (2) usa `read_exact` con un tamaño de buffer fijo en vez de leer "lo que venga" del socket TCP, ya que TCP no preserva límites de mensaje y puedes acabar leyendo un frame a medias o varios pegados.

### ⬜ Mini-Hito 4.2c — Hardening del protocolo *(nuevo, antes ausente)*
**Objetivo:** Que un paquete malformado o malicioso no pueda nunca crashear el emulador ni corromper el estado de la partida.

> Este es el hito que conecta directamente con tu preocupación de "que no entre nada por las conexiones". No se trata de criptografía compleja, sino de **disciplina de parsing** aplicada específicamente a la red:
> - **Define un formato de mensaje versionado y con longitud explícita** (ej. un byte de "tipo de mensaje" + 2 bytes de "longitud" + payload), en vez de asumir que cada paquete es "el siguiente byte SIO". Así puedes rechazar de forma determinista cualquier cosa que no encaje, sin ambigüedad.
> - **Nunca confíes en un campo de longitud declarado por el remitente sin comprobarlo contra el tamaño real recibido.** Si el campo dice "200 bytes" pero el paquete trae 4, debes detectarlo y descartar el mensaje, no leer 200 bytes de memoria que no están ahí.
> - **Limita la tasa de mensajes aceptados por segundo** de un mismo origen. Sin esto, un cliente (malicioso o simplemente con un bug en bucle) puede saturar tu Scheduler o tu cola de eventos de red, degradando la partida para el jugador legítimo o colgando el proceso.
> - **Nunca vuelques directamente bytes recibidos por red dentro de regiones de memoria emulada sensibles** (como vectores de interrupción o código) sin pasar por la misma validación de offsets que usarías para un archivo `.gba`. El Cable Link real solo mueve un byte de datos por transferencia (`SIODATA`); si tu protocolo de red hace algo más ambicioso (como sincronizar estados completos), ese camino merece la misma desconfianza que el parseo de la ROM en el Hito 1.2b.

**Tarea:** Implementar el formato de mensaje con longitud y tipo explícitos, validación de longitud declarada vs. recibida, y un límite simple de mensajes/segundo por conexión (puede ser tan simple como un contador con ventana de tiempo).
**Prueba:** Enviar manualmente (ej. con un script o `netcat`) paquetes truncados, con campos de longitud falseados, y a ritmo anormalmente alto, y comprobar que el emulador los descarta sin crashear ni ralentizarse de forma anómala.

#### ⬜ Hito 4.2b (más adelante, opcional) — Conexión P2P
**Objetivo:** Reducir latencia sustituyendo el relay por WebRTC o libp2p.
**Criterio de éxito:** Conexión directa entre jugadores.

> 🛡️ **Nota de seguridad si llegas a implementarlo:** al pasar a P2P pierdes el relay como punto intermedio que podría filtrar tráfico anómalo — la validación del Mini-Hito 4.2c pasa a ser la única línea de defensa real, así que confirma que sigue aplicándose íntegra sobre la conexión P2P y no solo sobre el camino del relay original.

### ⬜ Hito 4.3 — ¡Partida Online Funcional! (Lockstep Sync)
**Acción:** Algoritmo de sincronización por pasos (**Lockstep**):
1. El emulador corre hasta que el juego solicita una transferencia Cable Link.
2. Se **congela** por completo, esperando el paquete del otro emulador.
3. Al llegar el paquete, procesa el intercambio, avanza un paso y vuelve a congelarse.

> Esto solo deja de ser una experiencia de pausas insufribles si el contador de ciclos del Hito 2.2c es exacto: ambos emuladores deben ejecutar exactamente las mismas instrucciones en el mismo instante simulado, o aparecerá desync.

> 🛡️ **Seguridad — el riesgo específico del Lockstep es la denegación de servicio, no la corrupción:** como tu emulador se congela esperando el paquete del otro lado, un peer que deja de responder (a propósito o por un crash) cuelga tu partida indefinidamente. Añade un timeout razonable de espera; si se supera, desconecta limpiamente la sesión en vez de quedarte congelado para siempre. Es un caso de seguridad tan simple como importante: la disponibilidad también es seguridad.

**Criterio de éxito:** Jugar a distancia con un amigo, misma ROM (ej. Pokémon o Mario Kart Super Circuit), modo multijugador del juego, de forma estable.

---

## 📚 Recursos de referencia por fase

Un principiante en emulación necesita un "oráculo" — algo contra lo que comparar el comportamiento — porque la documentación oficial de Nintendo nunca se publicó. Estos son los que usa la comunidad:

- **GBATEK** (sitio de referencia técnica no oficial, mantenido por Martin Korth): la fuente más completa sobre el mapa de memoria, registros de hardware y formato de instrucciones. Cuando esta guía diga "implementa el registro X", GBATEK es donde miras los bits exactos.
- **`jsmolka/gba-tests`** (GitHub): los tests de CPU mencionados en el Mini-Hito 2.2b. Repositorio real y activo, contiene `arm.gba`, `thumb.gba` y tests de memoria/DMA por separado.
- **mGBA**: además de como emulador de referencia para comparar comportamiento, tiene un modo de logging de instrucción muy útil para diffear contra tu propia CPU instrucción a instrucción.
- **"The Rust Book"** (oficial, doc.rust-lang.org): si en algún hito el bloqueo es el lenguaje y no el hardware, este es el recurso a consultar antes que nada — especialmente los capítulos de ownership, enums/pattern matching (que usarás constantemente en el `decode`) y traits.

---

## ✅ Checkpoints de salud del proyecto

Revisa esto al terminar cada Fase, no cada mini-hito — es fácil perder de vista el bosque por los árboles en un proyecto tan largo.

- **Al terminar la Fase 1:** ¿Tu `gba_core` sigue compilando sin ninguna dependencia gráfica? Si `cargo build` en `gba_core` arrastra `minifb`, la separación ya se rompió — corrígelo antes de seguir, no después.
- **Al terminar la Fase 2.2:** ¿Cuántos tests de `arm.gba` pasan? Si no llegas al menos al 80%, no avances a BIOS/DMA/IRQ todavía — esos hitos asumen una CPU fiable y vas a estar depurando dos capas de bugs a la vez.
- **Al terminar la Fase 2.3:** ¿Tu emulador llega a mostrar el logo de Nintendo arrancando desde la BIOS real? Es el primer hito "visualmente honesto" — si no lo consigues, algo en BIOS/THUMB/timing está mal antes de meterte en gráficos serios.
- **Al terminar la Fase 2.4:** ¿Algún juego comercial real (no solo ROMs de test) llega a la pantalla de título? Este es el momento natural para hacer una pausa de celebración — has hecho la parte más difícil del proyecto.
- **Antes de empezar la Fase 4:** ¿Tu contador de ciclos coincide con valores de referencia conocidos (ej. de GBATEK o de logs de mGBA) para al menos una docena de instrucciones distintas? El Lockstep no perdona imprecisión aquí.

---

## 📌 Resumen de cambios en esta revisión

| Cambio | Razón |
|---|---|
| Sección de expectativas de tiempo + tabla por fase | Sin esto, el plan parece un sprint de fin de semana cuando son meses; saber dónde se suele abandonar ayuda a anticiparlo |
| Arquitectura `gba_core` / `gba_desktop` como decisión previa a la Fase 1 | Evita una refactorización masiva al portar a móvil |
| 2.1b marcado como atajo temporal, corregido en 2.3a | El arranque real de la GBA es en `0x0` (BIOS), no en la ROM |
| Nuevo Mini-Hito 2.1e (pipeline de 3 etapas) | Sin esto, los saltos calculados fallan de forma muy difícil de depurar |
| Aviso de registros banked por modo en 2.1a | Diseñar `Cpu` sin esto obliga a rehacer la estructura en el Hito 2.3c |
| Aviso de accesos desalineados (rotación de bytes) en 2.1a | Bug silencioso clásico: "funciona" en pruebas simples, falla con ROMs reales |
| Nuevo Mini-Hito 2.1c-bis (decode THUMB separado) | THUMB no es "ARM comprimido"; tratarlo como tal produce un decode mal mapeado |
| ROMs de test adelantadas a 2.2b (antes era 2.2d) | Validar la CPU cuanto antes evita implementar +100 instrucciones a ciegas |
| Aclaración del orden condición→opcode en 2.1c | La mayoría de instrucciones ARM reales son condicionales |
| Aviso de `KEYINPUT` invertido (activo-bajo) en Hito 3.1 | Inicializarlo a 0 hace que el juego crea que todas las teclas están pulsadas |
| Nota de tipos enteros explícitos para quien no viene de Rust | Los bugs de ancho de bit son la fuente de errores más común al empezar |
| Sección de recursos (GBATEK, gba-tests, mGBA, Rust Book) | Un principiante necesita saber dónde mirar, no solo qué implementar |
| Sección de checkpoints de salud por Fase | Detectar pronto si una fase no está lo bastante sólida para construir encima |
| Sección transversal de Principios de seguridad | Aclarar qué riesgo es real (bugs propios explotados por datos hostiles) vs. el que no existe (virus nativo en un `.gba`) |
| Validación de tamaño de ROM y parsing seguro de cabecera (1.2a, 1.2b) | Evitar panics o asignaciones descontroladas con archivos corruptos o malformados |
| Bus como punto único de validación de direcciones (2.1a) | Última línea de defensa ante direcciones inválidas generadas por ROMs corruptas o bugs propios |
| Nuevo Mini-Hito 4.2c (hardening de protocolo de red) | Es la fase de mayor riesgo real; sin esto, un peer hostil o con bugs puede crashear o colgar el emulador |
| Validación de paquetes y timeouts en Hitos 4.1–4.3 | Evitar panics por paquetes truncados/falseados y cuelgues indefinidos por peers caídos |
| Renumeración de 2.2b–2.4d | Para acomodar los hitos nuevos sin duplicar números |

---

*Documento generado a partir de la propuesta original y su revisión técnica ampliada.*
