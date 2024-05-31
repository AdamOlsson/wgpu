use core::time;
use std::time::Instant;
use cgmath::{num_traits::Pow, ElementWise, Vector3};
use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};
use super::util::{collision_detection::circle_line_segment_collision_detection, generate_random_colors, MAX_INSTANCES};

const BOTTOM_IDX: u8 = 0;
const LEFT_IDX: u8 = 1;
const RIGHT_IDX: u8 = 2;
const TOP_IDX: u8 = 3;

const X_AXIS: u8 = 0;
const Y_AXIS: u8 = 1;

const BOTTOM_LEFT_CORNER: Vector3<f32> = Vector3::new(-1.0, -1.0, 0.0);
const BOTTOM_RIGHT_CORNER: Vector3<f32> = Vector3::new(1.0, -1.0, 0.0);
const TOP_RIGHT_CORNER: Vector3<f32>= Vector3::new(1.0, 1.0, 0.0);
const TOP_LEFT_CORNER: Vector3<f32> = Vector3::new(-1.0, 1.0, 0.0);

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

        //let positions = generate_initial_positions_square_grid(num_instances, radius);
        let mut positions = [Vector3::new(0.0,0.8,0.0); MAX_INSTANCES];
        
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

        // Initial collision detection and response
        for i in 0..self.num_indices as usize {
            let pos = self.positions[i];
            let vel = self.velocities[i];
            let predicted_vel =  vel + self.g*timestep;
            let predicted_pos = pos + vel * timestep + 0.5*self.g*timestep.pow(2); 

            // Perform initial collision detection and response 
            let (border_idx, frac_to_collision) = Self::circle_border_collision_detection(
                &pos, &predicted_pos, self.radius);

            if border_idx == u8::MAX {
                // No collision
                self.positions[i] = predicted_pos;
                self.velocities[i] = predicted_vel; 
                continue
            }

            let ttc = frac_to_collision*timestep; // Time to collision
            let tac = timestep - ttc; // Time after collisiom
            let (new_pos , new_vel);
            match border_idx {
                BOTTOM_IDX => (new_pos, new_vel) = Self::circle_border_collision(&pos, &vel,&self.g, ttc, tac,Y_AXIS),
                LEFT_IDX => (new_pos, new_vel) = Self::circle_border_collision(&pos, &vel, &self.g, ttc, tac,X_AXIS),
                RIGHT_IDX => (new_pos, new_vel) = Self::circle_border_collision(&pos, &vel,&self.g, ttc, tac, X_AXIS),
                TOP_IDX => (new_pos, new_vel) = Self::circle_border_collision(&pos, &vel, &self.g, ttc, tac, Y_AXIS),
                _ => panic!("You should not be here!")
            }
            
            self.positions[i] = new_pos;
            self.velocities[i] = new_vel;
        }
    }

    fn circle_border_collision(
        pos: &Vector3<f32>, vel:  &Vector3<f32>, gravity: &Vector3<f32>,
        ttc: f32, tac: f32, reverse_dim: u8 
    ) -> (Vector3<f32>, Vector3<f32>) {
        // https://www.gamedev.net/forums/topic/571402-ball-never-stops-bouncing/4651063/
        // Compute pre-collision state, velocity and position
        let pre_collision_point = *pos + vel*ttc + 0.5*gravity*ttc.pow(2);
        let pre_collision_vel = vel + gravity*ttc;
        // Perform collision response
        let crf = Self::nonlinear_crf(
            f32::abs(pre_collision_vel.y), 0.85, 0.03, 0.7);
        let mut direction = Vector3::new(1.0, 1.0, 1.0);
        direction[reverse_dim as usize] = -crf;
        let post_collision_vel = vel.mul_element_wise(direction);
        // Compute post collision state
        let new_position = pre_collision_point + post_collision_vel*tac + 0.5 * gravity*tac.pow(2);
        let new_velocity = post_collision_vel + gravity * tac;
        return (new_position, new_velocity); 
    }


    /// Detects collision with any of the 4 borders.
    /// 
    /// Returns the index of the earliest border the circle collides with and the
    /// fraction of the timestep until collision occures, if there is a collision
    /// otherwise u8 max and 99.0.
    fn circle_border_collision_detection(
        pos: &Vector3<f32>, pred_pos: &Vector3<f32>, radius: f32
    ) -> (u8, f32) {
        // Check for collision with bottom boundary
        match circle_line_segment_collision_detection(
                &BOTTOM_LEFT_CORNER, &BOTTOM_RIGHT_CORNER,
                    pos, pred_pos, radius) {
            None => (), 
            Some(fraction_bottom) => 
                // Check for collision with right boundary
                match circle_line_segment_collision_detection(
                    &TOP_RIGHT_CORNER, &BOTTOM_RIGHT_CORNER,
                    pos, pred_pos, radius) {
                    Some(fraction_right) =>
                        if fraction_bottom < fraction_right {
                            return (BOTTOM_IDX, fraction_bottom)
                        } else {
                            return (RIGHT_IDX, fraction_right)
                        },
                    None => 
                        // Check for collision with left boundary
                        match circle_line_segment_collision_detection(
                            &TOP_LEFT_CORNER, &BOTTOM_LEFT_CORNER,
                            pos, pred_pos, radius) {
                            None => return (BOTTOM_IDX, fraction_bottom), 
                            Some(fraction_left) => 
                                if fraction_bottom < fraction_left {
                                    return (BOTTOM_IDX, fraction_bottom)
                                } else {
                                    return (LEFT_IDX, fraction_left)
                                },
                        },
                }
        } 

        // Check for collision with top boundary
        match circle_line_segment_collision_detection(
            &TOP_LEFT_CORNER, &TOP_RIGHT_CORNER,
                pos, pred_pos, radius) {
            None => (), 
            Some(fraction_top) => 
                // Check for collision with right boundary
                match circle_line_segment_collision_detection(
                    &TOP_RIGHT_CORNER, &BOTTOM_RIGHT_CORNER,
                    pos, pred_pos, radius) {
                    Some(fraction_right) =>
                        if fraction_top < fraction_right {
                            return (TOP_IDX, fraction_top)
                        } else {
                            return (RIGHT_IDX, fraction_right)
                        },
                    None => 
                        // Check for collision with left boundary
                        match circle_line_segment_collision_detection(
                            &TOP_LEFT_CORNER, &BOTTOM_LEFT_CORNER,
                            pos, pred_pos, radius) {
                            None => return (TOP_IDX, fraction_top), 
                            Some(fraction_left) => 
                                if fraction_top < fraction_left {
                                    return (TOP_IDX, fraction_top)
                                } else {
                                    return (LEFT_IDX, fraction_left)
                                },
                        },
                }
        } 
        return (u8::MAX, 99.0);
    }

}