
use core::num;
use std::{char::MAX, time::{Duration, Instant}};

use cgmath::{InnerSpace, Vector3};

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{generate_random_colors, MAX_INSTANCES};

pub struct VerletIntegration {
    pub positions: [Vector3<f32>; MAX_INSTANCES],
    prev_positions: [Vector3<f32>; MAX_INSTANCES],
    acceleration: [Vector3<f32>; MAX_INSTANCES],
    mass: [f32; MAX_INSTANCES],
    radius: [f32; MAX_INSTANCES],

    start_time: Instant,
    timestamp: Instant,

    pub num_instances: u32,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
    pub colors: [Vector3<f32>; MAX_INSTANCES],
}

impl VerletIntegration {
    pub fn new() -> Self {
        let common_radius = 0.05;
        let num_instances = 2;
        let mut positions = [Vector3::new(0.5, 0.0, 0.0); MAX_INSTANCES];
        positions[1] = Vector3::new(0.3, 0.3, 0.0);
        let prev_positions = positions.clone(); 
        let acceleration = [Vector3::new(0.0, -9.82, 0.0); MAX_INSTANCES];
        let mass = [1.0; MAX_INSTANCES];
        let radius = [common_radius; MAX_INSTANCES];

        let vertices = Circle::compute_vertices([0.0,0.0,0.0], common_radius);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;
        let colors = generate_random_colors();

        let timestamp = Instant::now();
        let start_time = Instant::now();

        Self {
            positions,
            prev_positions,
            acceleration,
            mass,
            radius,
            num_instances,
            vertices,
            indices,
            num_indices,
            colors,
            timestamp,
            start_time
        }
    }


    pub fn update(&mut self) {
        let now = Instant::now();
        let dt = (now - self.timestamp).as_millis() as f32 / 1000.0;
        self.timestamp = now;

        // Update positions
        for i in 0..self.num_instances as usize {
            let velocity = self.positions[i] - self.prev_positions[i];
            self.prev_positions[i] = self.positions[i];
            self.positions[i] = self.positions[i] + velocity + self.acceleration[i] * dt*dt;

        }

        // Constraints
        let constraint_center = Vector3::new(0.0,0.0,0.0);
        let constrain_radius = 0.9;
        for i in 0..self.num_instances as usize {
            let diff = self.positions[i] - constraint_center;
            let dist = diff.magnitude();
            if dist > (constrain_radius - self.radius[i]) {
                let correction_direction = diff / dist;
                self.positions[i] = constraint_center + correction_direction*(constrain_radius - self.radius[i]);
            }
        }
        
        // Solve collisions
        let num_substeps = 8;
        for _ in 0..num_substeps {
            for i in 0..self.num_instances as usize {
                let pos_a = self.positions[i];
                for j in (i+1)..self.num_instances as usize {
                    let pos_b = self.positions[j];
                    let collision_axis = pos_a - pos_b;
                    let dist = collision_axis.magnitude();
                    if dist < self.radius[i] + self.radius[j] {
                        let correction_direction = collision_axis / dist;
                        let collision_depth = self.radius[i] + self.radius[j] - dist;
                        self.positions[i] += 0.5*collision_depth*correction_direction;
                        self.positions[j] -= 0.5*collision_depth*correction_direction;
                    }
                }
            }
        }
    }
}