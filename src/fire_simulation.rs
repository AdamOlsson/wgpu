use crate::engine::renderer_engine::shapes::circle::Circle;

use std::{iter::zip, time::Instant};
use cgmath::{InnerSpace, MetricSpace, Vector3};
use crate::engine::{init_utils::{create_grid_positions, generate_random_radii}, physics_engine::{broadphase::{blockmap::BlockMap, BroadPhase}, collision::{CollisionBody, SimpleCollisionSolver}, constraint::{box_constraint::BoxConstraint, Constraint}, narrowphase::{Naive, NarrowPhase}}, renderer_engine::vertex::Vertex, Simulation, State};
use rayon::prelude::*;

const CIRCLE_CONTACT_SURFACE_AREA: f32 = 0.0002;
const BOTTOM_HEAT_SOURCE_TEMPERATURE: f32 = 3500.0;
const BOTTOM_HEAT_BOUNDARY: f32 = -0.985;
const HEAT_TRANSFER_COEFFICIENT: f32 = 0.028;
const BASE_GRAVITY: f32 = 1500.0;

const COMMON_RADIUS: f32 = 0.01;
const VELOCITY_CAP: f32 = 0.015;
const NUM_COLS: u32 = 50;
const NUM_ROWS: u32 = 50;
const INITIAL_SPACING: f32 = 0.005;

const COLOR_SPECTRUM_BUCKET_SIZE: f32 = 0.3;

const BLACK: Vector3<f32> = Vector3::new(31.0, 17.0, 15.0);
const RED: Vector3<f32> = Vector3::new(231.0, 24.0, 24.0); 
const ORANGE: Vector3<f32> = Vector3::new(231.0, 110.0, 24.0);
const YELLOW: Vector3<f32> = Vector3::new(249.0, 197.0, 26.0);
const WHITE: Vector3<f32> = Vector3::new(254.0, 244.0, 210.0);

pub struct FireState {
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>, 

    num_instances: u32,

    temperatures: Vec<f32>,
    bodies: Vec<CollisionBody>,
}

impl FireState {
    pub fn new(num_rows: u32, num_cols: u32, spacing: f32) -> Self {
        let initial_spacing = (COMMON_RADIUS * 2.0) + spacing;
        let initial_spacing_var = 0.001;
        let target_num_instances: u32 = num_rows * num_cols;

        let radii = generate_random_radii(target_num_instances, COMMON_RADIUS, 0.0);

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
    fn heat_conduction(temp1: f32, temp2: f32, distance: f32) -> (f32, f32) {
        let k = 1.0; // thermal conductivity
        let A = CIRCLE_CONTACT_SURFACE_AREA;
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
        HEAT_TRANSFER_COEFFICIENT*surface_area*(fluid_temp - object_temp)
    }


    fn heat_transfer(
        bodies: &Vec<CollisionBody>, temperatures: &Vec<f32>, dt: f32) -> Vec<f32> 
    {
        let broadphase = BlockMap::new();
        let candidates = broadphase.collision_detection(&bodies);
        let mut thermal_delta = vec![0.0; bodies.len()];

        let candidate_bodies: Vec<Vec<CollisionBody>> = candidates.iter().map(| cs | {
            cs.indices.iter().map(| idx | (bodies[*idx].clone())).collect()
        }).collect();

        let candidate_temperatures: Vec<Vec<f32>> = candidates.iter().map(| cs | {
            cs.indices.iter().map(| idx | (temperatures[*idx])).collect()
        }).collect();

        // This is still the most expensive operation of heat transfer by far.
        let thermal_deltas: Vec<Vec<f32>> = 
            candidate_bodies.par_iter()
            .zip(candidate_temperatures.par_iter())
            .map( | (bs, ts )| -> Vec<f32> {
                Self::local_heat_transfer(&bs, &ts, 0.0)
            })
            .collect();
        
        thermal_deltas.iter()
            .zip(candidate_bodies.iter())
            .for_each(|(ts, bs)| {
                bs.iter()
                    .enumerate()
                    .for_each(
                        |(i, b)| thermal_delta[b.id] += ts[i]
                    )
            });

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
            if bodies[i].position.y <= BOTTOM_HEAT_BOUNDARY {
                let (temp_delta_i, _) = Self::heat_conduction(temperatures[i], BOTTOM_HEAT_SOURCE_TEMPERATURE, bodies[i].radius);
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
        let state = FireState::new(NUM_ROWS, NUM_COLS, INITIAL_SPACING);
        let dt = 0.001;
        let color_spectrum = ColorSpectrum::new(
            vec![BLACK, RED, ORANGE, YELLOW, WHITE], COLOR_SPECTRUM_BUCKET_SIZE);

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
            state, dt, color_spectrum, timer, update_count,
            colors, indices, vertices, num_indices
        }
    }

    fn update(&mut self) {
        self.log_performance();
        let num_instances = self.state.num_instances;
        let bodies = &mut self.state.bodies;
        // Update positions
        for i in 0..num_instances as usize {
            let mut velocity = bodies[i].position - self.state.prev_positions[i];
            let vel_magn = velocity.magnitude();
            if vel_magn > VELOCITY_CAP {
                velocity = velocity*(VELOCITY_CAP/vel_magn)
            }

            self.state.prev_positions[i] = bodies[i].position;
            bodies[i].position = bodies[i].position + velocity + self.state.acceleration[i] * self.dt*self.dt;   
        }

        let constraint = BoxConstraint::new();
        let broadphase = BlockMap::new();
        let narrowphase = Naive::new();
        let collision_solver = SimpleCollisionSolver::new();

        for _ in 0..8 {
            // Constraint Application
            for i in 0..num_instances as usize {
                constraint.apply_constraint(&mut bodies[i]);
            }
            
            // Broadphase
            let candidates = broadphase.collision_detection(&bodies);

            // Narrowphase
            for c in candidates.iter() {
                narrowphase.collision_detection(bodies, c, &collision_solver);
            }
        }

        // Heat transfer
        let thermal_delta = Self::heat_transfer(&bodies, &self.state.temperatures, self.dt);
        
        let color_spectrum_len = self.color_spectrum.len();
        for (i, t) in thermal_delta.iter().enumerate() {
            self.state.temperatures[i] += t; 
            self.state.temperatures[i] = self.state.temperatures[i].max(0.0);
            let index = (self.state.temperatures[i] as usize).min(color_spectrum_len - 1);
            self.colors[i] = self.color_spectrum.get(index);
            self.state.acceleration[i].y = -BASE_GRAVITY + (self.state.temperatures[i].powi(2));
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

    fn get_vertices(&self) -> &Vec<Vertex> {
        &self.vertices
    }

    fn get_indices(&self) -> &Vec<u16> {
        &self.indices
    }

    fn get_colors(&self) -> &Vec<Vector3<f32>> {
        &self.colors
    }

    fn get_num_indices(&self) -> u32 {
        self.num_indices
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
    bucket_size: f32,
}
impl ColorSpectrum {
    pub fn new(key_colors: Vec<Vector3<f32>>, bucket_size: f32) -> Self {
        let mut spectrum = vec![];
        for i in 0..(key_colors.len() -1) {
            let color1 = key_colors[i];
            let color2 = key_colors[i+1];
            let colors = Self::interpolate(color1, color2, 100);
            spectrum.extend(colors);
        }
        Self {
            spectrum,
            bucket_size
        }
    }

    pub fn get(&self, index: usize) -> Vector3<f32> {
        let i = (index as f32 / self.bucket_size) as usize;
        self.spectrum[i.min(self.spectrum.len()-1)]
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