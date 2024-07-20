mod renderer_backend;
mod texture;
mod shapes;
mod game_engine;
mod engine;

use std::thread;

use winit::{
    dpi::PhysicalSize, event::*, event_loop::EventLoopBuilder, keyboard::{KeyCode, PhysicalKey}, window::WindowBuilder
};

#[derive(Debug, Clone, Copy)]
enum CustomEvent {
    Timer,
}

async fn run() {
    let event_loop = EventLoopBuilder::<CustomEvent>::with_user_event()
        .build()
        .unwrap();
    let event_loop_proxy = event_loop.create_proxy();
    let window = WindowBuilder::new().build(&event_loop).unwrap();
    let _ = window.request_inner_size(PhysicalSize::new(800, 800));

    let mut game_engine = game_engine::State::new(&window).await;

    std::thread::spawn(move || loop {
        // thread::sleep(std::time::Duration::from_millis(13));
        event_loop_proxy.send_event(CustomEvent::Timer).ok();
    });

    event_loop.run(
        move | event, elwt | match event {
            Event::UserEvent(..) => {
                game_engine.update();
                game_engine.render().unwrap();
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
                    game_engine.render().unwrap();
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

fn main() {
    pollster::block_on(run());
}