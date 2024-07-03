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
        let num_rows = 30;
        let num_cols = 30;
        let common_radius = 0.02;
        let initial_spacing = (common_radius * 2.0) + 0.01;
        let initial_spacing_var = 0.001;
        let target_num_instances: u32 = num_rows * num_cols;

        let radii = generate_random_radii(target_num_instances, common_radius, 0.0);

        let prev_positions = create_grid_positions(num_rows, num_cols, initial_spacing, Some(initial_spacing_var));
        let positions = prev_positions.clone();
        let acceleration = vec![Vector3::new(0.0, -150.0, 0.0); target_num_instances as usize];


        let bodies: Vec<CollisionBody> = zip(positions, radii).enumerate().map(|(i, (p, r))| CollisionBody::new(i, p, r)).collect();
        
        let temperatures = vec![0.0; target_num_instances as usize];

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
    const CIRCLE_CONTACT_SURFACE_AREA: f32 = 0.02;
    const BOTTOM_HEAT_SOURCE_TEMPERATURE: f32 = 50.0;
    const HEAT_TRANSFER_COEFFICIENT: f32 = 0.009;
    const BASE_GRAVITY: f32 = 500.0;

    const BLACK: Vector3<f32> = Vector3::new(31.0, 17.0, 15.0);
    const RED: Vector3<f32> = Vector3::new(231.0, 24.0, 24.0); 
    const ORANGE: Vector3<f32> = Vector3::new(231.0, 110.0, 24.0);
    const YELLOW: Vector3<f32> = Vector3::new(249.0, 197.0, 26.0);
    const WHITE: Vector3<f32> = Vector3::new(254.0, 244.0, 210.0);
    #[allow(non_snake_case)]
    fn heat_conduction(temp1: f32, temp2: f32, distance: f32) -> (f32, f32) {
        let k = 1.0; // thermal conductivity
        let A = Self::CIRCLE_CONTACT_SURFACE_AREA;
        let m1 = 1.0; // mass
        let m2 = 1.0; // mass
        let c1 = 1.0; // Heat capacity
        let c2 = 1.0; // Heat capacity
        let dT = (temp1 - temp2).abs();
        let dT1dt = (k*A*dT)/(distance.sqrt()*m1*c1);
        let dT2dt = (k*A*dT)/(distance.sqrt()*m2*c2);
        if temp1 > temp2 {
            return (-dT1dt, dT2dt);
        } else {
            return (dT1dt, -dT2dt);
        }
    }

    fn heat_convection(object_temp: f32, fluid_temp: f32, object_radius: f32) -> f32 {
        let surface_area = 2.0 * std::f32::consts::PI * object_radius; 
        Self::HEAT_TRANSFER_COEFFICIENT*surface_area*(fluid_temp - object_temp)
    }


    fn heat_transfer(
        bodies: &Vec<CollisionBody>, temperatures: &Vec<f32>, dt: f32) -> Vec<f32> 
    {
        let broadphase = SpatialSubdivision::new();
        let candidates = broadphase.collision_detection(&bodies);
        let mut thermal_delta = vec![0.0; bodies.len()];
        for cs in candidates {
            let mut candidate_bodies = vec![];
            let mut candidate_temperatures = vec![];
            
            for c in cs.indices.iter() {
                candidate_bodies.push(bodies[*c].clone());
                candidate_temperatures.push(temperatures[*c]);
            }
            let local_thermal_delta = Self::local_heat_transfer(&candidate_bodies, &candidate_temperatures, 0.0);
            
            for (k, c) in cs.indices.iter().enumerate() {
                thermal_delta[*c] += local_thermal_delta[k];
            }
        }
        return thermal_delta;
    }

    /// Calculate the heat transfer due to convection between bodies
    #[allow(non_snake_case)]
    fn local_heat_transfer(
        bodies: &Vec<CollisionBody>, temperatures: &Vec<f32>, dt: f32) -> Vec<f32> 
    {
        let num_instances = bodies.len();
        let mut thermal_delta = vec![0.0; num_instances ];

        for i in 0..num_instances {
            
            // Bottom of the screen heats the objects
            if bodies[i].position.y <= (-1.0 + bodies[i].radius) {
                let (temp_delta_i, _) = Self::heat_conduction(temperatures[i], Self::BOTTOM_HEAT_SOURCE_TEMPERATURE, bodies[i].radius);
                thermal_delta[i] += temp_delta_i; 
            } else {
                // Loose heat due to convection with air
                thermal_delta[i] += Self::heat_convection(temperatures[i], 0.0, bodies[i].radius);
            }

            for j in (i+1)..num_instances {
                // Heat conduction only happens between touching objects
                let dist_sq = bodies[i].position.distance2(bodies[j].position);
                if dist_sq -0.01 <= (bodies[i].radius + bodies[j].radius).powi(2) {
                    let (temp_delta_i, temp_delta_j) = Self::heat_conduction(temperatures[i], temperatures[j], bodies[i].radius + bodies[j].radius);
                    thermal_delta[i] += temp_delta_i;
                    thermal_delta[j] += temp_delta_j;
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
        let color_spectrum = ColorSpectrum::new(vec![
            Self::BLACK, Self::RED, Self::ORANGE, Self::YELLOW, Self::WHITE]);

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


        let mut avg_broadphase_time = 0.0;
        let mut avg_narrowphase_time = 0.0;
        let mut avg_constraint_time = 0.0;
        let mut avg_heat_transfer_time = 0.0;
        for _ in 0..3 {
            // Constraint Application
            let constraint_start = Instant::now();
            for i in 0..num_instances as usize {
                constraint.apply_constraint(&mut bodies[i]);
            }
            avg_constraint_time += constraint_start.elapsed().as_secs_f32();

            
            // Broadphase
            let broadphase_start = Instant::now();
            let candidates = broadphase.collision_detection(&bodies);
            avg_broadphase_time += broadphase_start.elapsed().as_secs_f32();

            // Narrowphase
            let narrowphase_start = Instant::now();
            for i in 0..candidates.len() {
                narrowphase.collision_detection(bodies, &candidates[i], &collision_solver);
            }
            avg_narrowphase_time += narrowphase_start.elapsed().as_secs_f32();
        }

        // Heat transfer
        let heat_transfer_start = Instant::now();
        let thermal_delta = Self::heat_transfer(&bodies, &self.state.temperatures, self.dt);
        avg_heat_transfer_time += heat_transfer_start.elapsed().as_secs_f32();


        
        let color_spectrum_len = self.color_spectrum.len();
        for (i, t) in thermal_delta.iter().enumerate() {
            self.state.temperatures[i] += t; 
            self.state.temperatures[i] = self.state.temperatures[i].max(0.0);
            let index = (self.state.temperatures[i] as usize).min(color_spectrum_len - 1);
            self.colors[i] = self.color_spectrum.get(index);
            //self.state.acceleration[i].y = -Self::BASE_GRAVITY + (self.state.temperatures[i].powi(2))/10.0;
        }

        println!("Avg Broadphase Time: {}", avg_broadphase_time/3.0);
        println!("Avg Narrowphase Time: {}", avg_narrowphase_time/3.0);
        println!("Avg Constraint Time: {}", avg_constraint_time/3.0);
        println!("Avg Heat Transfer Time: {}", avg_heat_transfer_time);
        println!("");
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