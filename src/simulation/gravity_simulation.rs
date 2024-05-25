use cgmath::Vector3;


use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{collision_detection::do_line_segments_intersect, generate_initial_positions_square_grid, generate_random_colors, MAX_INSTANCES};


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
        let g_vector = g_direction*g * 0.1;
        let timestep = 0.001; // 1 ms // TODO: Separate simulation timestep from framerate
        for i in 0..self.num_indices as usize {
            let new_velocity_y = self.velocities[i] + g_vector*timestep;
            self.velocities[i] = new_velocity_y;


            // Check for collision with bottom boundary
            let bottom_left_corner = Vector3::new(-1.0, -1.0, 0.0);
            let bottom_right_corner = Vector3::new(2.0, -1.0, 0.0);
            let start_pos = self.positions[i];
            let end_pos = start_pos + self.velocities[i]; // FIXME: Not correct as we need to take the radius into consideration
            match do_line_segments_intersect(bottom_left_corner, bottom_right_corner, start_pos, end_pos) {
                Some(_) => self.velocities[i].y *= -1.0, // TODO: Implement non-elastic collision response
                None => (),
            }

            self.positions[i] += self.velocities[i];
        }

    }
}