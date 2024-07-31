
use crate::engine::{game_engine, Simulation};
use winit::window::WindowBuilder;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::event_loop::EventLoopBuilder;
use winit::event::*;
use winit::dpi::PhysicalSize;

#[derive(Debug, Clone, Copy)]
enum CustomEvent {
    Timer,
}

pub async fn run<T: Simulation>(simulation: &mut T, window_size: PhysicalSize<u32>) {
    let event_loop = EventLoopBuilder::<CustomEvent>::with_user_event()
        .build()
        .unwrap();
    let event_loop_proxy = event_loop.create_proxy();
    let window = WindowBuilder::new().build(&event_loop).unwrap();
    let _ = window.request_inner_size(window_size);

    let mut game_engine = game_engine::GameEngine::new(&window, simulation).await;

    std::thread::spawn(move || loop {
        // thread::sleep(std::time::Duration::from_millis(13));
        event_loop_proxy.send_event(CustomEvent::Timer).ok();
    });

    event_loop.run(
        move | event, elwt | match event {
            Event::UserEvent(..) => {
                game_engine.update(simulation);
                game_engine.render(simulation).unwrap();
            }
            Event::WindowEvent {
                window_id,
                ref event,
            } if window_id == game_engine.window.id() => match event {
                WindowEvent::Resized(physical_size) => game_engine.resize(*physical_size),

                WindowEvent::CloseRequested
                | WindowEvent::KeyboardInput {
                    event:
                        KeyEvent {
                            physical_key: PhysicalKey::Code(KeyCode::Escape),
                            state: ElementState::Pressed,
                            repeat: false,
                            ..
                        },
                    ..
                } => {
                    println!("Goodbye, see you!");
                    elwt.exit();
                }

                WindowEvent::RedrawRequested => {
                    game_engine.render(simulation).unwrap();
                } 

                WindowEvent::KeyboardInput { event: 
                    KeyEvent {
                        state: ElementState::Pressed,
                        physical_key: PhysicalKey::Code(KeyCode::Space),
                        repeat: false,
                        ..
                    },
                    ..
                } => (),
                _ => (),
            },
            _ => ()
        }
    ).expect("Error!");
}

