use rand::Rng;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

const MAX_INSTANCES: usize = 10000;

pub struct CollisionSimulation {
    pub positions: [[f32;3]; MAX_INSTANCES],
    pub colors: [[f32;3]; MAX_INSTANCES],
    velocity_directons: [f32; MAX_INSTANCES],
    velocity_magnitudes: [f32; MAX_INSTANCES],

    pub num_instances: u32,
    radius: f32,

    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
}

impl CollisionSimulation {
    pub fn new() -> Self{
        let radius = 0.05;
        let num_instances: u32 = 100;
                
        let positions = CollisionSimulation::compute_initial_positions(num_instances, radius);
        let colors = CollisionSimulation::genrate_random_colors();

        let mut rng = rand::thread_rng();
    
        let mut velocity_magnitudes: [f32; MAX_INSTANCES] = [0.; MAX_INSTANCES];
        let mut velocity_directons: [f32; MAX_INSTANCES] = [0.; MAX_INSTANCES];
        for i in 0..num_instances as usize {
            velocity_magnitudes[i] = rng.gen_range(0.0..=0.01);
            velocity_directons[i] = rng.gen_range(0.0..=2.0*std::f32::consts::PI);
        }

        let vertices = Circle::compute_vertices([0.0, 0.0, 0.0], radius);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;

        Self { positions, colors, velocity_magnitudes, velocity_directons, num_instances, radius, indices, num_indices, vertices }
    }

    fn genrate_random_colors() -> [[f32;3]; MAX_INSTANCES] {
        let mut colors = [[0.0, 0.0, 0.0]; MAX_INSTANCES];
        let mut rng = rand::thread_rng();
        let min = 0.2;
        let max = 1.0;
        for i in 0..MAX_INSTANCES {
            colors[i] = [rng.gen_range(min..max), rng.gen_range(min..max), rng.gen_range(min..max)];
        }
        colors
    }

    fn collision_check(p1: [f32;3], p2: [f32;3], r1: f32, r2: f32) -> bool {
        let distance = f32::sqrt((p1[0] - p2[0]).powi(2) + (p1[1] - p2[1]).powi(2));
        distance <= r1 + r2
    }

    fn collosion_resolution(&mut self, i1: usize, i2: usize) {
        let p1 = self.positions[i1];
        let p2 = self.positions[i2];

        let distance = [p2[0] - p1[0],  p2[1] - p1[1]];
        let distance_mag = f32::sqrt(distance[0].powi(2) + distance[1].powi(2));
        let distance_unit = [distance[0] / distance_mag, distance[1] / distance_mag];
        let plane_norm = [-distance_unit[0], -distance_unit[1]];
        
        let direction = self.velocity_directons[i1];
        let direction_vec = [direction.cos(), direction.sin()];
        let dot_product = direction_vec[0] * plane_norm[0] + direction_vec[1] * plane_norm[1];
        let new_direction_vec = [
            direction_vec[0] - 2.0 * dot_product * plane_norm[0],
            direction_vec[1] - 2.0 * dot_product * plane_norm[1],
        ];

        let new_direction = new_direction_vec[1].atan2(new_direction_vec[0]);
        self.velocity_directons[i1] = new_direction;
    }

    pub fn update(&mut self) {
        // Check for collisions with other cirlces
        for i in 0..self.num_instances as usize {
            for j in 0..self.num_instances as usize {
                if i != j && CollisionSimulation::collision_check(self.positions[i], self.positions[j], self.radius, self.radius) {
                    self.collosion_resolution(i, j);
                }
            }
        }

        // Check for collision with boundaries
        for i in 0..self.num_instances as usize {
            if self.vertical_boundary_collision(i) {
                self.velocity_directons[i] = std::f32::consts::PI - self.velocity_directons[i];
            }
            if self.horizontal_boundary_collision(i) {
                self.velocity_directons[i] = 2.0*std::f32::consts::PI - self.velocity_directons[i];
            }

            let mag = self.velocity_magnitudes[i];
            let dir = self.velocity_directons[i];
            let x = self.positions[i][0];
            let y = self.positions[i][1];
            let x_new = x + mag * dir.cos();
            let y_new = y + mag * dir.sin();
            self.positions[i] = [x_new, y_new, 0.0];
            
        }
    }

    fn compute_initial_positions(num_instances: u32, radius: f32) -> [[f32;3]; MAX_INSTANCES]{
        let grid_side = f32::round(f32::sqrt(num_instances as f32) + 1.0);
        if grid_side * radius >= 2.0 {
            panic!("Radius too large for number of instances.");
        }

        let delta = 2.0 / grid_side;
        let mut positions = [[0.0, 0.0, 0.0]; MAX_INSTANCES];
        for i in 0..grid_side as usize {
            for j in 0..grid_side as usize {
                positions[i * grid_side as usize + j] = [
                    delta * i as f32 - 1.0 + delta / 2.0,
                    delta * j as f32 - 1.0 + delta / 2.0,
                    0.0,
                ];
            }
        }
        positions
    }

    fn vertical_boundary_collision(&mut self, i: usize) -> bool {
        let mag = self.velocity_magnitudes[i];
        let dir = self.velocity_directons[i];
        let x = self.positions[i][0];
        let x_new = x + mag * dir.cos();

        x_new + self.radius >= 1.0 || x_new - self.radius <= -1.0
    }

    fn horizontal_boundary_collision(&mut self, i: usize) -> bool {
        let mag = self.velocity_magnitudes[i];
        let dir = self.velocity_directons[i];
        let y = self.positions[i][1];
        let y_new = y + mag * dir.sin();
        y_new + self.radius >= 1.0 || y_new - self.radius <= -1.0
    }


}