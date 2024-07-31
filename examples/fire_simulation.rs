extern crate rust_wgpu;

use log::warn;
use rust_wgpu::engine::Simulation;
use rust_wgpu::examples::fire_simulation::FireSimulation;
use rust_wgpu::engine::run::run;
use winit::dpi::PhysicalSize;

fn main() {
    warn!("Fire simulation is not working as intended.");
    let window_size = PhysicalSize::new(800, 800);
    let mut simulation = FireSimulation::new(&window_size);
    pollster::block_on(run(&mut simulation, window_size));
}
