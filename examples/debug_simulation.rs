
extern crate rust_wgpu;

use rust_wgpu::engine::Simulation;
use rust_wgpu::examples::debug_simulation::DebugSimulation;
use rust_wgpu::engine::run::run;
use winit::dpi::PhysicalSize;

fn main() {
    let window_size = PhysicalSize::new(1000, 800);
    let mut simulation = DebugSimulation::new(&window_size);
    pollster::block_on(run(&mut simulation, window_size));
}
