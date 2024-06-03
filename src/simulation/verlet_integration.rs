

use std::time::Instant;

use cgmath::{InnerSpace, Vector3};

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::util::{generate_random_colors, MAX_INSTANCES};

pub struct VerletIntegration {
    pub positions: [Vector3<f32>; MAX_INSTANCES],
    prev_positions: [Vector3<f32>; MAX_INSTANCES],
    acceleration: [Vector3<f32>; MAX_INSTANCES],
    mass: [f32; MAX_INSTANCES],
    radius: [f32; MAX_INSTANCES],

    timestamp: Instant,
    last_spawn: Instant,

    pub num_instances: u32,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub num_instances_to_render: u32,
}

impl VerletIntegration {
    pub fn new() -> Self {
        let common_radius = 0.01;
        let num_instances = 5000;
        let num_instances_to_render = 1;
        let prev_positions = [Vector3::new(-0.5, 0.2, 0.0); MAX_INSTANCES];
        let positions = [Vector3::new(-0.48, 0.22, 0.0); MAX_INSTANCES];
        let acceleration = [Vector3::new(0.0, -9.82, 0.0); MAX_INSTANCES];
        let mass = [1.0; MAX_INSTANCES];
        let radius = [common_radius; MAX_INSTANCES];

        let vertices = Circle::compute_vertices([0.0,0.0,0.0], common_radius);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;
        let colors = generate_random_colors();

        let timestamp = Instant::now();
        let last_spawn = Instant::now();

        Self {
            positions,
            prev_positions,
            acceleration,
            mass,
            radius,
            num_instances,
            num_instances_to_render,
            vertices,
            indices,
            num_indices,
            colors,
            timestamp,
            last_spawn
        }
    }

    fn spawn(&mut self) {
        if self.num_instances == self.num_instances_to_render {
            return;
        }

        let now = Instant::now();
        let diff = now - self.last_spawn;
        if diff.as_millis() > 50 {
            self.num_instances_to_render += 1;
            self.last_spawn = now;
        }
    }

    pub fn update(&mut self) {
        self.spawn();
        let now = Instant::now();
        let dt = (now - self.timestamp).as_millis() as f32 / 1000.0;
        self.timestamp = now;

        let pre_update = Instant::now();
        // Update positions
        for i in 0..self.num_instances_to_render as usize {
            let velocity = self.positions[i] - self.prev_positions[i];
            self.prev_positions[i] = self.positions[i];
            self.positions[i] = self.positions[i] + velocity + self.acceleration[i] * dt*dt;   
        }
        let update_time = (Instant::now() - pre_update).as_millis();


        let pre_constraint= Instant::now();
        // Constraints
        let constraint_center = Vector3::new(0.0,0.0,0.0);
        let constrain_radius = 0.95;
        for i in 0..self.num_instances_to_render  as usize {
            let diff = self.positions[i] - constraint_center;
            let dist = diff.magnitude();
            if dist > (constrain_radius - self.radius[i]) {
                let correction_direction = diff / dist;
                self.positions[i] = constraint_center + correction_direction*(constrain_radius - self.radius[i]);
            }
        }
        let constraint_time = (Instant::now() - pre_constraint).as_millis();

        let pre_collision = Instant::now();
        // Solve collisions
        let num_substeps = 8;
        for _ in 0..num_substeps {
            for i in 0..self.num_instances_to_render as usize {
                let pos_a = self.positions[i];
                for j in (i+1)..self.num_instances_to_render as usize {
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
        let collision_time = (Instant::now() - pre_collision).as_millis();

        println!("dt: {} seconds, num_objects: {}", dt, self.num_instances_to_render);
        println!("update: {}, constraint: {}, collision: {}", update_time, constraint_time, collision_time);
    }
}