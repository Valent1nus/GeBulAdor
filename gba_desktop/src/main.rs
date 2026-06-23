//! # gba_desktop — frontend de escritorio del emulador
//!
//! Responsabilidades del frontend:
//! - Leer la ROM `.gba` del disco (I/O específico de escritorio) y entregársela
//!   al núcleo para que la valide.
//! - Abrir una ventana, pedirle al núcleo ([`gba_core`]) su framebuffer RGBA y
//!   pintarlo.
//!
//! Toda la lógica de emulación —y la decisión de qué es un cartucho válido—
//! vive en el núcleo; aquí solo hay I/O de plataforma y pintado.

use std::path::Path;

use gba_core::{
    Cartridge, Cpu, Decoded, Gba, Halt, RunStop, MAX_ROM_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH,
};
use minifb::{Key, Scale, Window, WindowOptions};

fn main() {
    // Modo arnés de test headless (Mini-Hito 2.2b), sin ventana:
    //   cargo run -p gba_desktop -- --test roms/arm.gba
    if std::env::args().nth(1).as_deref() == Some("--test") {
        match std::env::args().nth(2) {
            Some(path) => run_test_rom(Path::new(&path)),
            None => {
                eprintln!("Uso: gba_desktop --test <ruta-al-rom-de-test.gba>");
                std::process::exit(1);
            }
        }
        return;
    }

    // Primer argumento de la línea de comandos (opcional): la ruta a la ROM.
    //   cargo run -p gba_desktop -- "roms/Pokemon Rojo Fuego.gba"
    let gba = match std::env::args().nth(1) {
        Some(path) => match load_cartridge(Path::new(&path)) {
            Ok(cart) => {
                let size = cart.len();
                println!(
                    "Archivo cargado con éxito. Tamaño: {size} bytes ({}).",
                    human_size(size)
                );
                let header = cart.header();
                println!("  Título:       «{}»", header.title);
                println!("  Código juego: «{}»", header.game_code);

                // Mini-Hitos 2.1b/2.1c — Fetch + Decode: montamos la consola con
                // el cartucho (lo que coloca el PC en la ROM), leemos la primera
                // instrucción y la clasificamos (sin ejecutar su lógica todavía).
                let mut gba = Gba::with_cartridge(cart);
                let instr = gba.fetch();
                println!(
                    "  Primera instrucción ARM @ {:#010X}: {:#010X}",
                    gba.pc(),
                    instr
                );
                match gba.decode_arm(instr) {
                    Decoded::Execute(kind) => println!("  ¡Es una instrucción de {kind}!"),
                    Decoded::ConditionFailed(cond) => {
                        println!("  Condición {cond:?} no se cumple con el CPSR → se ignora (NOP).")
                    }
                }

                // Mini-Hito 2.1c-bis — Decode THUMB (16 bits, decoder separado del
                // de ARM). La GBA aún no ejecuta THUMB (el fetch real llega en
                // 2.3a), así que lo demostramos con una instrucción conocida:
                // 0x2005 = «MOV r0, #5» en THUMB.
                let thumb_ej: u16 = 0x2005;
                println!("  Ejemplo THUMB {thumb_ej:#06X} → {}", gba.decode_thumb(thumb_ej));

                // Mini-Hito 2.1d — Primera ejecución: la CPU altera un registro.
                // Demostración sintética sobre una CPU recién reseteada (la
                // primera instrucción de la ROM es un salto, aún no ejecutable):
                // «MOV R0, #5» (0xE3A00005) deja R0 = 5.
                let mut demo = Cpu::new();
                demo.execute_data_processing(0xE3A0_0005);
                println!("  Ejecución «MOV R0, #5» → R0 = {}", demo.reg(0));

                // Mini-Hito 2.1e — Pipeline de 3 etapas: al leer r15, una
                // instrucción ve el PC adelantado (+8 en ARM), no la dirección
                // donde está. Lo mostramos sobre una CPU situada en 0x08000000.
                let mut demo_pc = Cpu::new();
                demo_pc.set_pc(0x0800_0000);
                println!(
                    "  Pipeline: PC real {:#010X} → r15 visible {:#010X} (+{} en ARM)",
                    demo_pc.pc(),
                    demo_pc.reg(15),
                    demo_pc.reg(15) - demo_pc.pc()
                );

                // Mini-Hito 2.2a — Bucle de ejecución: corremos la CPU sobre la
                // ROM real hasta que se atasca en una instrucción aún no
                // implementada (o hasta un tope de seguridad contra bucles).
                println!("  ── Ejecutando (Mini-Hito 2.2a) ──");
                let report = gba.run(1_000_000);
                println!(
                    "  Instrucciones ejecutadas: {} ({} ciclos)",
                    report.steps, report.cycles
                );
                match report.stop {
                    RunStop::Halted(Halt::Unimplemented { pc, instr, kind }) => println!(
                        "  Detenida en {pc:#010X}: {instr:#010X} → {kind} (aún sin implementar)."
                    ),
                    RunStop::Halted(Halt::InfiniteLoop { pc, .. }) => {
                        println!("  Bucle infinito (b .) en {pc:#010X} — la CPU no avanza más.")
                    }
                    RunStop::Halted(Halt::ThumbNotImplemented { pc }) => {
                        println!("  Estado THUMB en {pc:#010X} — ejecución THUMB aún no implementada.")
                    }
                    RunStop::StepLimit => {
                        println!("  Tope de pasos alcanzado sin detenerse (¿bucle?).")
                    }
                }

                gba
            }
            Err(e) => {
                // Error legible y salida no-cero: nada de panics por un archivo
                // que el usuario podría haber tecleado mal o que esté corrupto.
                eprintln!("Error al cargar la ROM «{path}»: {e}");
                std::process::exit(1);
            }
        },
        None => {
            eprintln!("Uso: gba_desktop <ruta-al-rom.gba>");
            eprintln!("(No se indicó ROM; se abre solo la ventana de prueba.)");
            Gba::new()
        }
    };

    run_window(gba);
}

/// Arnés de test headless (Mini-Hito 2.2b): carga una ROM de test, la ejecuta
/// hasta que la CPU se detiene (bucle final `b .` o tope de pasos) y reporta el
/// veredicto leyendo `r12`, la convención de las gba-tests de jsmolka.
///
/// Mientras falten instrucciones por implementar, la CPU se detendrá antes de
/// llegar a ningún test (en el primer salto, por ejemplo); eso se reporta como
/// estado intermedio, no como veredicto. El primer paso para que avance es
/// Branch (Mini-Hito 2.2e).
fn run_test_rom(path: &Path) {
    let cart = match load_cartridge(path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error al cargar la ROM de test «{}»: {e}", path.display());
            std::process::exit(1);
        }
    };
    println!(
        "ROM de test «{}» ({} bytes). Ejecutando…",
        cart.header().title,
        cart.len()
    );

    // Tope de seguridad: una suite de test cabe de sobra; si se supera, algo va mal.
    const MAX_STEPS: u64 = 50_000_000;
    let mut gba = Gba::with_cartridge(cart);
    let report = gba.run(MAX_STEPS);
    println!(
        "Instrucciones ejecutadas: {} ({} ciclos)",
        report.steps, report.cycles
    );

    match report.stop {
        // Fin natural del test: r12 lleva el veredicto.
        RunStop::Halted(Halt::InfiniteLoop { pc, .. }) => {
            let r12 = gba.reg(12);
            if r12 == 0 {
                println!("✅ PASS — todos los tests pasaron (r12 = 0; bucle final en {pc:#010X}).");
            } else {
                println!("❌ FALLO en el test #{r12} (bucle final en {pc:#010X}).");
                std::process::exit(1);
            }
        }
        // Aún no es un veredicto: falta implementar la instrucción donde se paró.
        RunStop::Halted(Halt::Unimplemented { pc, instr, kind }) => {
            println!("⏸️  Detenida en {pc:#010X}: {instr:#010X} → {kind} (aún sin implementar).");
            println!(
                "    Se llegará más lejos según se implemente el resto del set ARM \
                 (data-processing con registro, cargas/almacenes, multiplicación...)."
            );
        }
        RunStop::Halted(Halt::ThumbNotImplemented { pc }) => {
            println!("⏸️  Estado THUMB en {pc:#010X}: la ejecución THUMB aún no está (2.2m/2.3a).");
        }
        RunStop::StepLimit => {
            println!("⏱️  Tope de {MAX_STEPS} pasos sin terminar (¿bucle no detectado o ROM muy larga?).");
        }
    }
}

/// Lee un fichero `.gba` del disco y lo convierte en un [`Cartridge`] validado.
///
/// Comprueba el tamaño en dos capas (defensa en profundidad):
/// 1. Por **metadatos**, ANTES de leer el fichero entero, para que un archivo
///    gigante no provoque una asignación de memoria enorme solo por abrirlo.
/// 2. Por **bytes reales**, dentro de [`Cartridge::from_bytes`], que es la
///    autoridad del núcleo sobre qué es un cartucho válido.
fn load_cartridge(path: &Path) -> Result<Cartridge, Box<dyn std::error::Error>> {
    let declared_size = std::fs::metadata(path)?.len();
    if declared_size > MAX_ROM_SIZE as u64 {
        return Err(format!(
            "el archivo mide {declared_size} bytes; el máximo de la GBA es {MAX_ROM_SIZE} \
             bytes (32 MiB). No se carga."
        )
        .into());
    }

    // `std::fs::read` es la forma idiomática de `File::open` + `read_to_end`.
    let bytes = std::fs::read(path)?;

    // El núcleo revalida y se queda con la propiedad de los bytes.
    let cart = Cartridge::from_bytes(bytes)?;
    Ok(cart)
}

/// Da formato legible a un tamaño en bytes (KiB / MiB).
fn human_size(bytes: usize) -> String {
    const KIB: usize = 1024;
    const MIB: usize = 1024 * 1024;
    if bytes >= MIB {
        format!("{:.2} MiB", bytes as f64 / MIB as f64)
    } else if bytes >= KIB {
        format!("{:.2} KiB", bytes as f64 / KIB as f64)
    } else {
        format!("{bytes} bytes")
    }
}

/// Abre la ventana y ejecuta el bucle de pintado del framebuffer del núcleo.
fn run_window(gba: Gba) {
    // El núcleo (`gba`) produce los píxeles; el frontend solo los muestra.

    // minifb pinta desde un buffer de `u32` en formato 0RGB (0x00RRGGBB). El
    // núcleo entrega RGBA en bytes, así que mantenemos un buffer de conversión.
    let mut buffer: Vec<u32> = vec![0; SCREEN_WIDTH * SCREEN_HEIGHT];

    let mut window = Window::new(
        "EmulaRUST — GBA (Fase 2.2a · ESC para salir)",
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        WindowOptions {
            // 240×160 es diminuto en pantallas modernas; ×4 → 960×640 visible
            // sin cambiar la resolución real del framebuffer.
            scale: Scale::X4,
            ..WindowOptions::default()
        },
    )
    .expect("No se pudo crear la ventana de minifb");

    // ~60 FPS: sin esto el bucle giraría al 100 % de un núcleo del host.
    window.set_target_fps(60);

    while window.is_open() && !window.is_key_down(Key::Escape) {
        rgba_to_0rgb(gba.framebuffer(), &mut buffer);
        window
            .update_with_buffer(&buffer, SCREEN_WIDTH, SCREEN_HEIGHT)
            .expect("No se pudo actualizar la ventana");
    }
}

/// Convierte un framebuffer RGBA (4 bytes/píxel) al formato 0RGB (`0x00RRGGBB`)
/// empaquetado en `u32` que espera minifb.
fn rgba_to_0rgb(rgba: &[u8], out: &mut [u32]) {
    for (pixel, slot) in rgba.chunks_exact(4).zip(out.iter_mut()) {
        let r = pixel[0] as u32;
        let g = pixel[1] as u32;
        let b = pixel[2] as u32;
        // El canal alfa (pixel[3]) se ignora: la ventana es totalmente opaca.
        *slot = (r << 16) | (g << 8) | b;
    }
}
