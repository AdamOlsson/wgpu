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
    spawn_rate_ms: u32,

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
        let num_instances = 10000;
        let spawn_rate_ms = 50;
        let num_instances_to_render = 0;
        let prev_positions = [Vector3::new(-0.5, 0.2, 0.0); MAX_INSTANCES];
        let positions = [Vector3::new(-0.48, 0.22, 0.0); MAX_INSTANCES];
        let acceleration = [Vector3::new(0.0, -18.82, 0.0); MAX_INSTANCES];
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
            last_spawn,
            spawn_rate_ms
        }
    }

    fn spawn(&mut self, fps: f32) {
        if self.num_instances == self.num_instances_to_render {
            return;
        }

        let target_fps = 60.0;
        if self.num_instances_to_render > 1 && fps < target_fps {
            //panic!("Fps is below {}", target_fps);
            return;
        }

        let now = Instant::now();
        let diff = now - self.last_spawn;
        if diff.as_millis() > self.spawn_rate_ms as u128 {
            self.num_instances_to_render += 1;
            self.last_spawn = now;
        }
    }

    pub fn update(&mut self) {
        let now = Instant::now();
        //let dt = (now - self.timestamp).as_millis() as f32 / 1000.0;
        let dt = 0.001;
        let actual_time = (now - self.timestamp).as_millis() as f32 / 1000.0;
        let fps = 1.0/actual_time;
        self.timestamp = now;
        self.spawn(fps);

        // Update positions
        for i in 0..self.num_instances_to_render as usize {
            let velocity = self.positions[i] - self.prev_positions[i];
            self.prev_positions[i] = self.positions[i];
            self.positions[i] = self.positions[i] + velocity + self.acceleration[i] * dt*dt;   
        }

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

        // Solve collisions
        let num_substeps = 1;
        let pos_slice = &mut self.positions[0..self.num_instances_to_render as usize];
        let rad_slice = &mut self.radius[0..self.num_instances_to_render as usize];
        for _ in 0..num_substeps {
            Self::spatial_subdivision(pos_slice, rad_slice, self.num_instances_to_render);
        }

        println!("fps: {} seconds, num_objects: {}", fps, self.num_instances_to_render);
    }

    fn naive_collision_detection_and_resolution(
        positions: &mut [Vector3<f32>], radius: &[f32], num_instances: u32
    ){
        for i in 0..num_instances as usize {
            let pos_a = positions[i];
            for j in (i+1)..num_instances as usize {
                let pos_b = positions[j];
                let collision_axis = pos_a - pos_b;
                let dist = collision_axis.magnitude();
                if dist == 0.0 {
                    panic!("Collision axis has zero length");
                }
                if dist < radius[i] + radius[j] {
                    let correction_direction = collision_axis / dist;
                    let collision_depth = radius[i] + radius[j] - dist;
                    positions[i] += 0.5*collision_depth*correction_direction;
                    positions[j] -= 0.5*collision_depth*correction_direction;
                }
            }
        }
    }

    fn spatial_subdivision(
        positions: &mut [Vector3<f32>], radius: &[f32], num_instances: u32
    ) {
        if num_instances == 0 {
            return;
        }
        // Create grid with largest side equal to the largest diameter of the circles
        let cell_size = radius.iter().take(num_instances as usize).fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
        let grid_width = (2.0/cell_size).ceil() as u32;

        // Assign each circle to a cell
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); (grid_width*grid_width) as usize];
        for i in 0..num_instances as usize {
            let pos = positions[i];
            // Add 1.0 to offset all coordinates between 0.0 and 2.0
            let x = ((pos.x + 1.0)/cell_size) as u32;
            let y = ((pos.y + 1.0)/cell_size) as u32;
            let cell_index = (y*grid_width + x) as usize;
            cells[cell_index].push(i);
        }
        // For each cell, compute collision between all circles in the current cell and
        // all surrounding cells. Skip over the outer most cells.
        for i in 1..(grid_width-1) {
            for j in 1..(grid_width-1){
                let center_cell = i*grid_width + j;
                let local_cell_ids = Self::get_neighboring_cell_ids(center_cell as u32, grid_width);
                
                // Collect local positions and radii
                let mut local_positions: Vec<Vector3<f32>> = Vec::new();
                let mut local_radius: Vec<f32> = Vec::new();
                let mut local_object_ids: Vec<usize> = Vec::new();
                for cell_id in local_cell_ids.iter() {
                    for object_id in cells[*cell_id as usize].iter() {
                        local_positions.push(positions[*object_id].clone());
                        local_radius.push(radius[*object_id].clone());
                        local_object_ids.push(*object_id);
                    }
                }

                let num_local_instances = local_positions.len() as u32;
                if num_local_instances <= 1 {
                    continue;
                }
                Self::naive_collision_detection_and_resolution(&mut local_positions, &local_radius, num_local_instances);
                // Update positions
                for k in 0..num_local_instances as usize {
                    positions[local_object_ids[k] as usize] = local_positions[k];
                }
            }
        }
    }
    fn get_neighboring_cell_ids(center_id: u32, grid_width: u32) -> [u32; 9] {
        let top_left = center_id - grid_width - 1;
        let top_center = center_id - grid_width;
        let top_right = center_id - grid_width + 1;
        let center_left = center_id - 1;
        let center_right = center_id + 1;
        let bottom_left = center_id + grid_width - 1;
        let bottom_center = center_id + grid_width;
        let bottom_right = center_id + grid_width + 1;
        return [
            top_left, top_center, top_right,
            center_left, center_id, center_right,
            bottom_left, bottom_center, bottom_right
        ];
    }
}