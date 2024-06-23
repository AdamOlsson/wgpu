use std::iter::zip;

use cgmath::Vector3;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle};

use super::{broadphase::{BroadPhase, SpatialSubdivision}, collision::{CollisionBody, SimpleCollisionSolver}, constraint::{BoxConstraint, Constraint}, engine::Engine, init_utils::{create_grid_positions, generate_random_colors, generate_random_radii}, narrowphase::{Naive, NarrowPhase}, State};

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
        let num_rows = 10;
        let num_cols = 10;
        let common_radius = 0.05;
        let target_num_instances: u32 = num_rows * num_cols;

        let radii = generate_random_radii(target_num_instances, common_radius, 0.0);

        let prev_positions = create_grid_positions(num_rows, num_cols, 0.11);
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

        let constraint = BoxConstraint::new();
        let broadphase = SpatialSubdivision::new();
        let narrowphase = Naive::new();
        let collision_solver = SimpleCollisionSolver::new();

        let num_instances = bodies.len(); // FIXME: Use states num_instances
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