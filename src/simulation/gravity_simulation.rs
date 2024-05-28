use std::time::Instant;

use cgmath::{num_traits::Pow, ElementWise, InnerSpace, MetricSpace, Vector3};


use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{collision_detection::{circle_line_segment_collision_detection, do_line_segments_intersect}, collision_response::circle_circle_collision_response, generate_initial_positions_square_grid, generate_random_colors, MAX_INSTANCES};


pub struct GravitySimulation {
    pub positions: [Vector3<f32>; MAX_INSTANCES],
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub mass: [f32; MAX_INSTANCES],
    velocities: [Vector3<f32>; MAX_INSTANCES],

    pub num_instances: u32,
    radius: f32,
    g: Vector3<f32>,
    timestamp: Instant,

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

        let g = Vector3::new(0.0,-1.0,0.0) * 9.82; // m/s2
        let timestamp = Instant::now();

        Self { positions, colors, mass, velocities, num_instances, radius, g, timestamp, indices, num_indices, vertices }
    }

    fn nonlinear_crf(impact_velocity: f32, e_0: f32, k: f32, n:f32) -> f32 {
        let e =  e_0 - k * impact_velocity.powf(n);
        return f32::max(0.0, e);
    }

    // https://stackoverflow.com/questions/17471241/bounce-velocity-calculation-collision-with-ground
    pub fn update(&mut self) {
        let now = Instant::now();
        let timestep = (now - self.timestamp).as_millis() as f32 / 1000.0;
        self.timestamp = now;

        // Accelerate due to gravity
        let g_vector = self.g * timestep;
        for i in 0..self.num_indices as usize {
            self.velocities[i] += g_vector;
            
            // Check for collision with bottom boundary
            let bottom_left_corner = Vector3::new(-1.0, -1.0, 0.0);
            let bottom_right_corner = Vector3::new(2.0, -1.0, 0.0);
            let pos = self.positions[i];
            let predicted_pos = pos + self.velocities[i] * timestep; 
            match circle_line_segment_collision_detection(bottom_left_corner, bottom_right_corner, pos, predicted_pos, self.radius) {
                None => self.positions[i] += self.velocities[i] * timestep,
                Some(fraction) => {
                    // https://www.gamedev.net/forums/topic/571402-ball-never-stops-bouncing/4651063/
                    let ttc = timestep*fraction;
                    // Compute pre-collision state, velocity and position
                    let pre_collision_point = self.positions[i] + self.velocities[i]*ttc + 0.5*self.g*ttc.pow(2);
                    let pre_collision_vel = self.velocities[i] + self.g*ttc;
                    
                    // Perform collision response
                    let crf = Self::nonlinear_crf(
                        f32::abs(pre_collision_vel.y), 0.85, 0.03, 0.7);

                    let post_collision_vel = self.velocities[i].mul_element_wise(Vector3::new(0.0, -crf, 0.0));
                    
                    // Compute post collision state
                    let tac = timestep - ttc; // Time After Collision
                    let new_position = pre_collision_point + post_collision_vel*tac + 0.5 * self.g*tac.pow(2);
                    let new_velocity = post_collision_vel + self.g * tac;
                    
                    self.positions[i] = new_position;
                    self.velocities[i] = new_velocity;
                }
            }
            
        }
        
    }
}