use std::iter::zip;

use cgmath::Vector3;
use rand::Rng;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::{broadphase::{BroadPhase, SpatialSubdivision}, collision::{CollisionBody, SimpleCollisionSolver}, constraint::{CircleConstraint, Constraint}, engine::Engine, init_utils::generate_random_colors, narrowphase::{Naive, NarrowPhase}, State};

pub trait Simulation {
    fn new() -> Self;
    fn update(&mut self);
    fn get_bodies(&self) -> &Vec<CollisionBody>;
    fn get_positions(&self) -> &Vec<Vector3<f32>>;
    fn get_radii(&self) -> &Vec<f32>;
    fn get_num_active_instances(&self) -> u32;
    fn get_target_num_instances(&self) -> u32;
}

pub struct FireState {
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>, 

    num_instances: u32,

    heat: Vec<f32>,
    bodies: Vec<CollisionBody>,
}

impl FireState {
    pub fn new() -> Self {
        let num_rows = 1;
        let num_cols = 2;
        let common_radius = 0.05;
        let target_num_instances: u32 = num_rows * num_cols;

        let radii = Self::generate_random_radii(target_num_instances, common_radius, 0.0);

        let prev_positions = Self::create_grid_positions(num_rows, num_cols, 0.5);
        let positions = prev_positions.clone();
        let acceleration = vec![Vector3::new(0.0, -150.0, 0.0); target_num_instances as usize];

        let bodies: Vec<CollisionBody> = zip(positions, radii).map(|(p, r)| CollisionBody::new(p, r)).collect();

        Self {
            bodies,
            prev_positions,
            acceleration,
            num_instances: target_num_instances,
            heat: vec![],
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
    engine: Engine,
    state: FireState,
    dt: f32,

    // FIXME: Eventually move these out of the simulation
    pub colors: Vec<Vector3<f32>>,
    pub indices: Vec<u16>,
    pub vertices: Vec<Vertex>,
    pub num_indices: u32,
}

impl Simulation for FireSimulation {
    fn new() -> Self {
        let engine = Engine::new();
        let state = FireState::new();
        let dt = 0.001;
        let colors = generate_random_colors(state.get_bodies().len() as u32);
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0], 1.0);
        let num_indices = (359)*3;
        Self {
            engine, state, dt,
            colors, indices, vertices, num_indices
        }
    }

    fn update(&mut self) {
        let state = &mut self.state; 
        let bodies = &mut state.bodies;
        // Update positions
        for i in 0..state.num_instances as usize {
            let velocity = bodies[i].position -  state.prev_positions[i];
            state.prev_positions[i] = bodies[i].position;
            bodies[i].position = bodies[i].position + velocity + state.acceleration[i] * self.dt*self.dt;   
        }


        // Apply constraints
        let constraint = CircleConstraint::new();
        let num_instances = bodies.len(); // FIXME: Use states num_instances
        for i in 0..num_instances as usize {
            constraint.apply_constraint(&mut bodies[i]);
        }
        //Engine::constraint_handler(&mut self.state, &constraint);

        // Collision Detection
        let broadphase = SpatialSubdivision::new();
        let candidates = broadphase.collision_detection(&self.state);
        //let candidates = Engine::broad_phase_collision_detection(&self.state, &broadphase);
        let narrowphase = Naive::new();
        let collision_solver = SimpleCollisionSolver::new();
        for i in 0..candidates.len() {
            narrowphase.collision_detection(&mut self.state, &candidates[i], &collision_solver);
        }
        //Engine::narrow_phase_collision_detection(&mut self.state, &candidates, &narrowphase, &collision_solver);

        // TODO: Perform broad and narrow phase and find out which bodies are touching
        //let touching_candidates = Engine::broad_phase_collision_detection(&self.state, &broadphase); 
        // perform specific body interactions
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