use std::time::Instant;

use cgmath::Vector3;
use rand::Rng;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::{util::{generate_random_colors, MAX_INSTANCES}, verlet_integration::VerletIntegration};


pub struct ParticleFireSimulation{
    engine: VerletIntegration,
    
    target_num_instances: u32,

    last_update: Instant,
    dt: f32,

    // FIXME: Eventually move these out of the simulation
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub indices: Vec<u16>,
    pub vertices: Vec<Vertex>,
    pub num_indices: u32,
}

impl ParticleFireSimulation {
    pub fn new() -> Self {
        let num_rows = 10;//32;
        let num_cols = 10;//29;
        let common_radius = 0.03;
        let target_num_instances: u32 = num_rows * num_cols;
        
        // FIXME: Add render support for different sized radii
        let mut radii = vec![common_radius; target_num_instances as usize];
        radii[0] = 0.08;

        let prev_positions = Self::create_grid_positions(num_rows, num_cols, 0.01);
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

        let colors = generate_random_colors();
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], common_radius);
        let num_indices = (359)*3;

        Self {
            engine,
            target_num_instances,
            last_update,
            dt,
            colors,
            indices,
            vertices,
            num_indices,
        }
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
        
        println!("fps: {} seconds, num_objects: {}", fps, self.target_num_instances);
    }

    pub fn get_positions(&self) -> &Vec<Vector3<f32>> {
        self.engine.get_positions()
    }

    pub fn get_num_active_instances(&self) -> u32 {
        self.target_num_instances
    }

    pub fn get_target_num_instances(&self) -> u32 {
        self.target_num_instances
    }
}