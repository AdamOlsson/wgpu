use cgmath::Vector3;


use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{generate_initial_positions_square_grid, generate_random_colors, MAX_INSTANCES};


pub struct GravitySimulation {
    pub positions: [Vector3<f32>; MAX_INSTANCES],
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub mass: [f32; MAX_INSTANCES],
    velocities: [Vector3<f32>; MAX_INSTANCES],

    pub num_instances: u32,
    radius: f32,

    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
}

impl GravitySimulation {
    pub fn new() -> Self{
        let radius = 0.1;
        let num_instances: u32 = 1;

        let positions = generate_initial_positions_square_grid(num_instances, radius);
        
        let velocities = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];

        let colors = generate_random_colors();
        let mass = [1.0; MAX_INSTANCES];
        let vertices = Circle::compute_vertices([0.0, 0.0, 0.0], radius);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;

        Self { positions, colors, mass, velocities, num_instances, radius, indices, num_indices, vertices }
    }

    pub fn update(&mut self) {

        // Accelerate due to gravity
        let g = 9.82; // m/s2
        let g_direction = Vector3::new(0.0,-1.0,0.0);
        let g_vector = g_direction*g;
        let timestep = 0.001; // 1 ms // TODO: Separate simulation timestep from framerate
        for i in 0..self.num_indices as usize {
            let new_velocity_y = self.velocities[i] + g_vector*timestep;
            self.velocities[i] = new_velocity_y;


            // TODO: Check for collision with bottom boundary

            // TODO: Perform collision response against bottom boundary, include inelastic collision

            self.positions[i] += self.velocities[i];
        }

    }
}