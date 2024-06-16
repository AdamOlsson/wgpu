use std::time::Instant;

use cgmath::{MetricSpace, Vector3};
use rand::Rng;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::verlet_integration::VerletIntegration;


pub struct ParticleFireSimulation{
    engine: VerletIntegration,
    
    target_num_instances: u32,

    last_update: Instant,
    dt: f32,

    temperature: Vec<f32>,

    // FIXME: Eventually move these out of the simulation
    pub colors: Vec<Vector3<f32>>,
    pub indices: Vec<u16>,
    pub vertices: Vec<Vertex>,
    pub num_indices: u32,
}

impl ParticleFireSimulation {
    pub fn new() -> Self {
        let num_rows = 32;
        let num_cols = 29;
        let common_radius = 0.03;
        let target_num_instances: u32 = num_rows * num_cols;
        
        let radii = Self::generate_random_radii(target_num_instances, common_radius, 0.0);

        let prev_positions = Self::create_grid_positions(num_rows, num_cols, 0.062);
        let positions = prev_positions.clone();
        let acceleration = vec![Vector3::new(0.0, -150.0, 0.0); target_num_instances as usize];

        let mut engine = VerletIntegration::new();
        engine.use_constraint(super::verlet_integration::box_constraint);
        engine.use_solver(super::verlet_integration::SolverType::SpatialSubdivision);
        engine.set_num_solver_steps(3);
        engine.set_radii(radii);
        engine.set_prev_positions(prev_positions);
        engine.set_positions(positions);
        engine.set_acceleration(acceleration);
        engine.spawn_all_instances();

        let last_update = Instant::now();
        let dt = 0.001;
        
        let mut temperature = vec![0.0; target_num_instances as usize];
        let mut colors = vec![];
        for i in 0..temperature.len() {
            if i % num_cols as usize == 0 {
                temperature[i] = 1000.0;
                colors.push(Vector3::new( f32::clamp(temperature[i], 0.0, 255.0), 0.0, 0.0));
            } else {
                colors.push(Vector3::new( 0.0, 0.0, 0.0));
            }
        }
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], 1.0);
        let num_indices = (359)*3;

        Self {
            engine,
            target_num_instances,
            last_update,
            dt,
            temperature,
            colors,
            indices,
            vertices,
            num_indices,
        }
    }

    fn generate_random_radii(num_instances: u32, base_radius: f32, variance: f32) -> Vec<f32> {
        if variance == 0.0 {
            return vec![base_radius; num_instances as usize];
        }
        let mut rng = rand::thread_rng();
        let mut radii = Vec::new();
        for _ in 0..num_instances {
            let radius = base_radius + rng.gen_range(-variance.abs()..variance.abs());
            radii.push(radius);
        }
        return radii;
    }

    fn create_grid_positions(num_rows: u32, num_cols: u32, spacing: f32) -> Vec<Vector3<f32>> {
        let mut positions = Vec::new();
        let mut rng = rand::thread_rng();
        for i in 0..num_rows {
            for j in 0..num_cols {
                let mut x = (i as f32 - num_rows as f32 / 2.0) * spacing;
                let y = (j as f32 - num_cols as f32 / 2.0) * spacing;
                x += rng.gen_range(-0.01..0.01);
                positions.push(Vector3::new(x, y, 0.0));
            }
        }
        positions
    }

    pub fn update(&mut self) {
        self.engine.update(self.dt);

        let now = Instant::now();
        let time_diff = now - self.last_update;
        let fps = 1.0 / time_diff.as_secs_f32();
        self.last_update = now;


        let positions = self.engine.get_positions();
        let radii = self.engine.get_radii();
        let num_instances = positions.len();
        let mut total_temperature_delta = vec![0.0; positions.len()];
        let temp_delta = 1.0/10000.0;
        for i in 0..num_instances {
            let pos_a = positions[i];
            let rad_a = radii[i];
            for j in (i+1)..num_instances {
                let pos_b = positions[j];
                let rad_b = radii[j];
                let dist = pos_a.distance2(pos_b);
                if dist <= (rad_a + rad_b).powi(2) {
                    // transfer heat
                    if self.temperature[i] < self.temperature[j] {
                        total_temperature_delta[i] += temp_delta;
                        total_temperature_delta[j] -= temp_delta;
                    } else if self.temperature[i] > self.temperature[j] {
                        total_temperature_delta[i] -= temp_delta;
                        total_temperature_delta[j] += temp_delta;
                    }
                }
            }
        }
        for (i, t) in total_temperature_delta.iter().enumerate() {
            self.temperature[i] += t;
            self.colors[i] = Vector3::new(self.temperature[i].clamp(0.0, 255.0) as f32, 0.0, 0.0);
        }

        println!("fps: {} seconds, num_objects: {}", fps, self.target_num_instances);
    }

    pub fn get_positions(&self) -> &Vec<Vector3<f32>> {
        self.engine.get_positions()
    }

    pub fn get_radii(&self) -> &Vec<f32> {
        self.engine.get_radii()
    }

    pub fn get_num_active_instances(&self) -> u32 {
        self.target_num_instances
    }

    pub fn get_target_num_instances(&self) -> u32 {
        self.target_num_instances
    }
}