use std::{iter::zip, time::Instant};
use cgmath::{MetricSpace, Vector3};
use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};
use super::{broadphase::{BroadPhase, SpatialSubdivision}, collision::{CollisionBody, SimpleCollisionSolver}, constraint::{BoxConstraint, Constraint}, engine::Engine, init_utils::{create_grid_positions, generate_random_radii}, narrowphase::{Naive, NarrowPhase}, State};

pub trait Simulation {

    fn new() -> Self;
    fn update(&mut self);
    fn get_bodies(&self) -> &Vec<CollisionBody>;
    fn get_positions(&self) -> &Vec<Vector3<f32>>;
    fn get_radii(&self) -> &Vec<f32>;
    fn get_num_active_instances(&self) -> u32;
    fn get_target_num_instances(&self) -> u32;

    fn log_performance(&mut self);
}

pub struct FireState {
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>, 

    num_instances: u32,

    temperatures: Vec<f32>,
    bodies: Vec<CollisionBody>,
}

impl FireState {
    pub fn new() -> Self {
        let num_rows = 10;
        let num_cols = 10;
        let common_radius = 0.05;
        let target_num_instances: u32 = num_rows * num_cols;

        let radii = generate_random_radii(target_num_instances, common_radius, 0.0);

        let prev_positions = create_grid_positions(num_rows, num_cols, 0.11);
        let positions = prev_positions.clone();
        let acceleration = vec![Vector3::new(0.0, -150.0, 0.0); target_num_instances as usize];

        let bodies: Vec<CollisionBody> = zip(positions, radii).map(|(p, r)| CollisionBody::new(p, r)).collect();
        
        let mut temperatures = vec![0.0; target_num_instances as usize];
        for i in 0..temperatures.len() {
            if i % num_cols as usize == 0 {
                temperatures[i] = 1000.0;
            }
        }

        Self {
            bodies,
            prev_positions,
            acceleration,
            num_instances: target_num_instances,
            temperatures,
        }
    }
}

impl State for FireState {
    fn get_bodies(&self) -> &Vec<CollisionBody> {
        &self.bodies
    }
    fn get_bodies_mut(&mut self) -> &mut Vec<CollisionBody> {
        &mut self.bodies
    }
}

pub struct FireSimulation {
    // Simulation information
    engine: Engine,
    state: FireState,
    dt: f32,

    // Performance information
    timer: Instant,
    update_count: u32,

    // Render information 
    color_spectrum: ColorSpectrum,
    pub colors: Vec<Vector3<f32>>,
    pub indices: Vec<u16>,
    pub vertices: Vec<Vertex>,
    pub num_indices: u32,
}

impl FireSimulation {
    #[allow(non_snake_case)]
    fn conduct_heat(bodies: &Vec<CollisionBody>, temperatures: &Vec<f32>, dt: f32) -> Vec<f32> {
        let num_instances = bodies.len();
        let k = 1.0; // thermal conductivity
        let A = 0.1; // cross sectional area of touch
        let m1 = 1.0; // mass
        let m2 = 1.0; // mass
        let c1 = 1.0; // Heat capacity
        let c2 = 1.0; // Heat capacity
        let mut thermal_delta = vec![0.0; num_instances ];
        for i in 0..num_instances {
            for j in (i+1)..num_instances {
                let dist = bodies[i].position.distance2(bodies[j].position);
                if dist <= (bodies[i].radius + bodies[j].radius).powi(2) {
                    let dT = temperatures[i] - temperatures[j];
                    let dT1dt = -(k*A*dT)/(dist.sqrt()*m1*c1);
                    let dT2dt = (k*A*dT)/(dist.sqrt()*m2*c2);
                    thermal_delta[i] += dT1dt*dt;
                    thermal_delta[j] += dT2dt*dt;
                }
            }
        }
        return thermal_delta;
    }
}

impl Simulation for FireSimulation {
    fn new() -> Self {
        let engine = Engine::new();
        let state = FireState::new();
        let dt = 0.001;
        let black = Vector3::new(31.0, 17.0, 15.0);
        let red = Vector3::new(231.0, 24.0, 24.0); 
        let orange = Vector3::new(231.0, 110.0, 24.0);
        let yellow = Vector3::new(249.0, 197.0, 26.0);
        let white = Vector3::new(254.0, 244.0, 210.0);
        let color_spectrum = ColorSpectrum::new(vec![black, red, orange, yellow, white]);

        let mut colors = vec![color_spectrum.get(0); state.num_instances as usize];
        let color_spectrum_len = color_spectrum.len();
        for i in 0..state.num_instances as usize {
            let index = (state.temperatures[i] as usize).min(color_spectrum_len - 1);
            colors[i] = color_spectrum.get(index);
        }

        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], 1.0);
        let num_indices = (359)*3;

        let timer = Instant::now();
        let update_count = 0;

        Self {
            engine, state, dt, color_spectrum, timer, update_count,
            colors, indices, vertices, num_indices
        }
    }

    fn update(&mut self) {
        self.log_performance();
        let num_instances = self.state.num_instances;
        let bodies = &mut self.state.bodies;
        // Update positions
        for i in 0..num_instances as usize {
            let velocity = bodies[i].position -  self.state.prev_positions[i];
            self.state.prev_positions[i] = bodies[i].position;
            bodies[i].position = bodies[i].position + velocity + self.state.acceleration[i] * self.dt*self.dt;   
        }

        let constraint = BoxConstraint::new();
        let broadphase = SpatialSubdivision::new();
        let narrowphase = Naive::new();
        let collision_solver = SimpleCollisionSolver::new();

        for _ in 0..3 {
            for i in 0..num_instances as usize {
                constraint.apply_constraint(&mut bodies[i]);
            }
        
            // Collision Detection
            let candidates = broadphase.collision_detection(&bodies);
            for i in 0..candidates.len() {
                narrowphase.collision_detection(bodies, &candidates[i], &collision_solver);
            }
        }

        // Heat transfer
        let thermal_delta = Self::conduct_heat(&bodies, &self.state.temperatures, self.dt);
        let color_spectrum_len = self.color_spectrum.len();
        for (i, t) in thermal_delta.iter().enumerate() {
            self.state.temperatures[i] += f32::max(*t, 0.0);
            let index = (self.state.temperatures[i] as usize).min(color_spectrum_len - 1);
            self.colors[i] = self.color_spectrum.get(index); 
        }
    }

    fn log_performance(&mut self) {
        self.update_count += 1;
        let now = Instant::now();
        let diff = now.duration_since(self.timer);
        if diff.as_millis() > 1000 {
            let fps = self.update_count / diff.as_secs() as u32; 
            println!("fps: {}", fps);
            self.update_count = 0;
            self.timer = now;
        }
    }

    fn get_bodies(&self) -> &Vec<CollisionBody> {
        self.state.get_bodies()
    }

    fn get_positions(&self) -> &Vec<Vector3<f32>> {
        todo!();
    }

    fn get_radii(&self) -> &Vec<f32> {
       todo!();
    }

    fn get_num_active_instances(&self) -> u32 {
       self.state.num_instances 
    }

    fn get_target_num_instances(&self) -> u32 {
        self.state.num_instances
    }
}

struct ColorSpectrum {
    spectrum: Vec<Vector3<f32>>,
}
impl ColorSpectrum {
    pub fn new(key_colors: Vec<Vector3<f32>>) -> Self {
        let mut spectrum = vec![];
        for i in 0..(key_colors.len() -1) {
            let color1 = key_colors[i];
            let color2 = key_colors[i+1];
            let colors = Self::interpolate(color1, color2, 100);
            spectrum.extend(colors);
        }
        Self {
            spectrum
        }
    }

    pub fn get(&self, index: usize) -> Vector3<f32> {
        self.spectrum[index]
    }

    pub fn len(&self) -> usize {
        self.spectrum.len()
    }
    fn interpolate(color1: Vector3<f32>, color2: Vector3<f32>, num_steps: u32) -> Vec<Vector3<f32>> {
        let mut colors = vec![];
        for i in 0..num_steps {
            let t = i as f32 / num_steps as f32;
            let color = (color1 + (color2 - color1) * t)/255.0;
            colors.push(color);
        }
        colors
    }
}