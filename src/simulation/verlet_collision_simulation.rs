use std::time::{Duration, Instant};
use cgmath::Vector3;
use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle, simulation::solvers::spatial_subdivision::spatial_subdivision_solver};
use super::{circle_constraint, util::{generate_random_colors, MAX_INSTANCES}};

pub struct VerletCollisionSimulation {
    positions: Vec<Vector3<f32>>,
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>, 
    radii: Vec<f32>,
    
    num_solver_steps: u32,

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
        let num_solver_steps = 1;

        let last_update = Instant::now();
        let last_spawn = Instant::now();
        let dt = 0.001;

        let colors = generate_random_colors();
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], 1.0);
        let num_indices = (359)*3;

        Self {
            positions,
            prev_positions,
            acceleration,
            radii,
            num_solver_steps,
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
        &self.positions
    }

    pub fn get_radii(&self) -> &Vec<f32> {
        &self.radii
    }

    pub fn get_num_active_instances(&self) -> u32 {
        self.num_active_instances
    }

    pub fn get_target_num_instances(&self) -> u32 {
        self.target_num_instances
    }


    pub fn update(&mut self) {
        // Update positions
        for i in 0..self.num_active_instances as usize {
            let velocity = self.positions[i] - self.prev_positions[i];
            self.prev_positions[i] = self.positions[i];
            self.positions[i] = self.positions[i] + velocity + self.acceleration[i] * self.dt*self.dt;   
        }

        // Constraints
        for i in 0..self.num_active_instances  as usize {
            circle_constraint(&mut self.positions[i], &self.radii[i]);
        }
        
        // Solve collisions
        let pos_slice = &mut self.positions[0..self.num_active_instances as usize];
        let rad_slice = &mut self.radii[0..self.num_active_instances as usize];
        for _ in 0..self.num_solver_steps {
            spatial_subdivision_solver( pos_slice, rad_slice);
        }

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
            self.num_active_instances += 1;
            self.last_spawn = now;
        }
    }
}