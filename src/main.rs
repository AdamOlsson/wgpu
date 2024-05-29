mod renderer_backend;
mod util;
mod texture;
mod world;
mod shapes;
mod state;
mod simulation;


use std::{thread, time::Instant};

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

    let mut state = state::State::new(&window).await;

    std::thread::spawn(move || loop {
        //thread::sleep(std::time::Duration::from_millis(100));
        event_loop_proxy.send_event(CustomEvent::Timer).ok();
    });

    event_loop.run(
        move | event, elwt | match event {
            Event::UserEvent(..) => {
                let start = Instant::now();
                state.update();
                let duration = start.elapsed();
                state.render().unwrap();
            }
            Event::WindowEvent {
                window_id,
                ref event,
            } if window_id == state.window.id() => match event {
                WindowEvent::Resized(physical_size) => state.resize(*physical_size),

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
                    state.render().unwrap();
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