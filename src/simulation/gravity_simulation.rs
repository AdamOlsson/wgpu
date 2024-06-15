use core::panic;
use std::time::Instant;
use cgmath::{num_traits::Pow, ElementWise, InnerSpace, Vector3};
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

const ZERO_VELOCITY_THRESHOLD: f32 = 0.1;

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
        let mut positions = [Vector3::new(0.0,-0.6,0.0); MAX_INSTANCES];
        
        let velocities = [Vector3::new(1.0, 0.0, 0.0); MAX_INSTANCES];

        let colors = generate_random_colors();
        let mass = [1.0; MAX_INSTANCES];
        let vertices = Circle::compute_vertices([0.0, 0.0, 0.0], 1.0);
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

    fn round_to_3_decimals(value: f32) -> f32 {
        return (value*1000.0).round() / 1000.0;
    }

    // https://stackoverflow.com/questions/17471241/bounce-velocity-calculation-collision-with-ground
    pub fn update(&mut self) {
        let now = Instant::now();
        // TODO: Make sure that we never increment past the timestep
        let timestep = (now - self.timestamp).as_millis() as f32 / 1000.0;
        self.timestamp = now;

        for i in 0..self.num_instances as usize {
            //println!("\nInstance: {}", i);
            let pos = self.positions[i];
            let vel = self.velocities[i];
            let predicted_vel = vel + self.g*timestep;
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

            let ttc = Self::round_to_3_decimals(frac_to_collision*timestep); // Time to collision
            let tac = Self::round_to_3_decimals(timestep - ttc); // Time after collisiom
            let (new_pos , new_vel);
            match border_idx {
                BOTTOM_IDX => {
                    if vel.y == 0.0 {
                        // Object is resting at bottom border;
                        new_pos = pos + vel*timestep;
                        new_vel = vel;
                    } else {
                        (new_pos, new_vel) = Self::elastic_circle_border_collision(&pos, &vel, &self.g, ttc, tac, Y_AXIS);
                    }
                },
                LEFT_IDX => (new_pos, new_vel) = Self::elastic_circle_border_collision(&pos, &vel, &self.g, ttc, tac,X_AXIS),
                RIGHT_IDX => (new_pos, new_vel) = Self::elastic_circle_border_collision(&pos, &vel,&self.g, ttc, tac, X_AXIS),
                TOP_IDX => (new_pos, new_vel) = Self::elastic_circle_border_collision(&pos, &vel, &self.g, ttc, tac, Y_AXIS),
                _ => panic!("You should not be here!")
            }

            self.positions[i] = new_pos;
            self.velocities[i] = new_vel;
        }
    }

    fn elastic_circle_border_collision(
        pos: &Vector3<f32>, vel:  &Vector3<f32>, gravity: &Vector3<f32>,
        ttc: f32, tac: f32, reverse_dim: u8 
    ) -> (Vector3<f32>, Vector3<f32>) {
        let tbc = ttc - 0.001; // Time before collision
        // https://www.gamedev.net/forums/topic/571402-ball-never-stops-bouncing/4651063/
        // Compute pre-collision state, velocity and position
        let pre_collision_pos = pos + vel*tbc + 0.5*gravity*tbc.pow(2);
        let pre_collision_vel = vel + gravity*tbc;
        // Perform collision response
        let crf = Self::nonlinear_crf(
            f32::abs(pre_collision_vel.y), 0.85, 0.03, 0.7);
        let mut direction = Vector3::new(1.0, 1.0, 1.0);
        direction[reverse_dim as usize] = -crf;
        let post_collision_vel = pre_collision_vel.mul_element_wise(direction);
        // Compute post collision state
        let mut new_position = pre_collision_pos + post_collision_vel*tac + 0.5 * gravity*tac.pow(2);
        let gravity_vec = gravity*tac;
        let mut new_velocity = post_collision_vel + gravity_vec;
        if gravity_vec.y.abs() > post_collision_vel.y.abs() {
            // Note: when the gravity is stronger than the bounce up, the objects new
            // position will fall under the bottom border. To prevent this, we simply
            // set the y-component of the new velocity to zero.
            new_velocity.y = 0.0;
            let mask = Vector3::new(1.0, 0.0, 1.0);
            new_position = pos + vel.mul_element_wise(mask)*tbc;
        }
        return (new_position, new_velocity); 
    }

    fn elastic_collision(
        crf: f32, vel_a: f32, vel_b: f32, mass_a: f32, mass_b:f32
    ) -> f32 {
        return ( crf*mass_b*(vel_b-vel_a) + mass_a*vel_a + mass_b*vel_b) / (mass_a + mass_b);
    }

    /// Detects collision with any of the 4 borders.
    ///
    /// The function is optmized to only evaluate 3 borders and make use of the fact
    /// that if a collision occurs the a border, there is no need to check for collision
    /// with the opposite border.
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

        // Check for collision with right boundary
        match circle_line_segment_collision_detection(
            &TOP_RIGHT_CORNER, &BOTTOM_RIGHT_CORNER,
                pos, pred_pos, radius) {
            None => (), 
            Some(fraction_right) => 
                // Check for collision with top boundary
                match circle_line_segment_collision_detection(
                    &TOP_LEFT_CORNER, &TOP_RIGHT_CORNER,
                    pos, pred_pos, radius) {
                    Some(fraction_top) =>
                        if fraction_right < fraction_top {
                            return (RIGHT_IDX, fraction_right)
                        } else {
                            return (TOP_IDX, fraction_top)
                        },
                    None => 
                        // Check for collision with bottom boundary
                        match circle_line_segment_collision_detection(
                                &BOTTOM_LEFT_CORNER, &BOTTOM_RIGHT_CORNER,
                                pos, pred_pos, radius) {
                            None => return (RIGHT_IDX, fraction_right), 
                            Some(fraction_bottom) => 
                                if fraction_right < fraction_bottom {
                                    return (RIGHT_IDX, fraction_right)
                                } else {
                                    return (BOTTOM_IDX, fraction_bottom)
                                },
                        },
                }
        } 

        // Check for collision with left boundary
        match circle_line_segment_collision_detection(
            &TOP_LEFT_CORNER, &BOTTOM_LEFT_CORNER,
                pos, pred_pos, radius) {
            None => (), 
            Some(fraction_left) => 
                // Check for collision with top boundary
                match circle_line_segment_collision_detection(
                        &TOP_LEFT_CORNER, &TOP_RIGHT_CORNER,
                        pos, pred_pos, radius) {
                    Some(fraction_top) =>
                        if fraction_left < fraction_top {
                            return (LEFT_IDX, fraction_left)
                        } else {
                            return (TOP_IDX, fraction_top)
                        },
                    None => 
                        // Check for collision with bottom boundary
                        match circle_line_segment_collision_detection(
                                &BOTTOM_LEFT_CORNER, &BOTTOM_RIGHT_CORNER,
                                pos, pred_pos, radius) {
                            None => return (LEFT_IDX, fraction_left), 
                            Some(fraction_bottom) => 
                                if fraction_left < fraction_bottom {
                                    return (LEFT_IDX, fraction_left)
                                } else {
                                    return (BOTTOM_IDX, fraction_bottom)
                                },
                        },
                }
        } 
        return (u8::MAX, 99.0);
    }


}