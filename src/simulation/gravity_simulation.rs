use std::time::Instant;

use cgmath::{num_traits::Pow, ElementWise, InnerSpace, MetricSpace, Vector3};


use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{collision_detection::{circle_line_segment_collision_detection, do_line_segments_intersect}, collision_response::circle_circle_collision_response, generate_initial_positions_square_grid, generate_random_colors, MAX_INSTANCES};

const VELOCITY_ZERO_THRESH: f32 = 0.0001;
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
        let gravity_vector = self.g * timestep;
        for i in 0..self.num_indices as usize {
            let pos = self.positions[i];
            let vel = self.velocities[i];
            let predicted_vel =  vel + gravity_vector;
            let predicted_pos = pos + predicted_vel * timestep; 
    
            let (new_pos, new_vel) = Self::border_collision_detection_and_resolution(
                &pos, &predicted_pos,
                &vel, &predicted_vel,
                self.radius, timestep, &self.g);
            
            self.positions[i] = new_pos;
            self.velocities[i] = new_vel;
        }
    }

    /// Perform collision detection and response with all 4 boundaries.
    /// 
    /// Args:
    /// - pos: the position of the circle at the start of the timestep.
    /// - pred_pos: the predicted position if no collision had occured.
    /// - vel: the velocity of the circle at the start of the timestep.
    /// - pred_vel: the predicted velocity if no collisions occured.
    /// - radius: the radius of the circle.
    /// - timestep: size of the timestep in milliseconds.
    /// - gravity: the gravity vector.
    /// 
    /// Returns the new position and velocity after collision response.
    fn border_collision_detection_and_resolution(
        pos: &Vector3<f32>, pred_pos: &Vector3<f32>,
        vel:  &Vector3<f32>, pred_vel: &Vector3<f32>,
        radius: f32, timestep: f32, gravity: &Vector3<f32>
    ) -> (Vector3<f32>, Vector3<f32>){
        // Check for collision with bottom boundary
        let bottom_left_corner = Vector3::new(-1.0, -1.0, 0.0);
        let bottom_right_corner = Vector3::new(2.0, -1.0, 0.0);
        match circle_line_segment_collision_detection(
                &bottom_left_corner, &bottom_right_corner,
                    pos, pred_pos, radius) {
            None => return (*pred_pos, *pred_vel), 
            Some(fraction) => {

                if -VELOCITY_ZERO_THRESH < vel.y && vel.y < VELOCITY_ZERO_THRESH {
                    let new_vel = Vector3::new(vel.x, 0.0, vel.z);
                    return (*pos, new_vel); // TODO: Might not be right. Consider a ball rolling and hitting a vertical boundary
                } 
                // https://www.gamedev.net/forums/topic/571402-ball-never-stops-bouncing/4651063/
                let ttc = timestep*fraction;
                // Compute pre-collision state, velocity and position
                let pre_collision_point = *pos + (*vel)*ttc + 0.5*gravity*ttc.pow(2);
                let pre_collision_vel = *vel + gravity*ttc;
                
                // Perform collision response
                let crf = Self::nonlinear_crf(
                    f32::abs(pre_collision_vel.y), 0.85, 0.03, 0.7);

                let post_collision_vel = vel.mul_element_wise(Vector3::new(0.0, -crf, 0.0));
                
                // Compute post collision state
                let tac = timestep - ttc; // Time After Collision
                let new_position = pre_collision_point + post_collision_vel*tac + 0.5 * gravity*tac.pow(2);
                let new_velocity = post_collision_vel + gravity * tac;

                return (new_position, new_velocity);
            }
        }

    }
}