use std::time::{Duration, Instant};
use cgmath::Vector3;
use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};
use super::{util::{generate_random_colors, MAX_INSTANCES}, verlet_integration::VerletIntegration};

pub struct VerletCollisionSimulation {
    engine: VerletIntegration,
    
    num_active_instances: u32,
    target_num_instances: u32,

    last_update: Instant,
    last_spawn: Instant,
    spawn_rate: Duration,
    dt: f32,

    // FIXME: Eventually move these out of the simulation
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub indices: Vec<u16>,
    pub vertices: Vec<Vertex>,
    pub num_indices: u32,
}

impl VerletCollisionSimulation {
    pub fn new() -> Self {
        let target_num_instances = 1000;
        let num_active_instances = 0;
        let common_radius = 0.01;
        let spawn_rate = Duration::from_millis(50);
        
        let radii = [common_radius; MAX_INSTANCES].to_vec();
        let prev_positions = [Vector3::new(-0.5, 0.2, 0.0); MAX_INSTANCES].to_vec();
        let positions = [Vector3::new(-0.48, 0.22, 0.0); MAX_INSTANCES].to_vec();
        let acceleration = [Vector3::new(0.0, -150.0, 0.0); MAX_INSTANCES].to_vec();

        let mut engine = VerletIntegration::new();
        engine.use_constraint(super::verlet_integration::circle_constraint);
        engine.use_solver(super::verlet_integration::SolverType::SpatialSubdivision);
        engine.set_num_solver_steps(1);
        engine.set_radii(radii);
        engine.set_prev_positions(prev_positions);
        engine.set_positions(positions);
        engine.set_acceleration(acceleration);

        let last_update = Instant::now();
        let last_spawn = Instant::now();
        let dt = 0.001;

        let colors = generate_random_colors();
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], 1.0);
        let num_indices = (359)*3;

        Self {
            engine,
            target_num_instances,
            num_active_instances,
            spawn_rate,
            last_spawn,
            dt,
            last_update,
            colors,
            indices,
            vertices,
            num_indices,
        }
    }

    pub fn get_positions(&self) -> &Vec<Vector3<f32>> {
        self.engine.get_positions()
    }

    pub fn get_radii(&self) -> &Vec<f32> {
        self.engine.get_radii()
    }

    pub fn get_num_active_instances(&self) -> u32 {
        self.num_active_instances
    }

    pub fn get_target_num_instances(&self) -> u32 {
        self.target_num_instances
    }


    pub fn update(&mut self) {
        self.engine.update(self.dt);

        if self.num_active_instances == self.target_num_instances {
            return;
        }

        let now = Instant::now();
        let time_diff = now - self.last_update;
        let fps = 1.0 / time_diff.as_secs_f32();
        self.last_update = now; 
        
        let target_fps = 60.0;
        if self.num_active_instances > 1 && fps < target_fps {
            return;
        }

        self.spawn(now);        
        println!("fps: {} seconds, num_objects: {}", fps, self.num_active_instances);
    }

    fn spawn(&mut self, now: Instant) {
        let spawn_diff = now - self.last_spawn;
        if spawn_diff > self.spawn_rate {
            self.engine.spawn_one_instance();
            self.num_active_instances += 1;
            self.last_spawn = now;
        }
    }
}