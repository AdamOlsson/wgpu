pub mod init_utils;
pub mod physics_engine;
pub mod renderer_engine;
pub mod game_engine;

use cgmath::Vector3;
use physics_engine::collision::CollisionBody;
use renderer_engine::vertex::Vertex;

pub trait State {
    fn get_bodies(&self) -> &Vec<CollisionBody>;
    fn get_bodies_mut(&mut self) -> &mut Vec<CollisionBody>;
}

pub trait Simulation {
    fn new() -> Self;
    fn update(&mut self);
    fn get_bodies(&self) -> &Vec<CollisionBody>;
    fn get_positions(&self) -> &Vec<Vector3<f32>>;
    fn get_radii(&self) -> &Vec<f32>;
    fn get_vertices(&self) -> &Vec<Vertex>;
    fn get_indices(&self) -> &Vec<u16>;
    fn get_colors(&self) -> &Vec<Vector3<f32>>;
    fn get_num_active_instances(&self) -> u32;
    fn get_target_num_instances(&self) -> u32;
    fn get_num_indices(&self) -> u32;

    fn log_performance(&mut self);
}

