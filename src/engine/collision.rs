
use cgmath::{InnerSpace, Vector3};

use super::State;

pub struct CollisionBody {
    pub position: Vector3<f32>,
    pub radius: f32
}

impl CollisionBody {
    pub fn new(position: Vector3<f32>, radius: f32) -> Self {
        Self {
            position,
            radius,
        }
    }
}

pub struct CollisionPair {
    pub body_1: usize,
    pub body_2: usize
}

impl CollisionPair {
    pub fn new() -> Self {
        Self {
            body_1: 0,
            body_2: 1,
        }
    }
}

pub struct CollisionCandidates {
//    candidates: Vec<&'a CollisionBody>
    pub indices: Vec<usize>
}

impl CollisionCandidates {
    pub fn new(indices: Vec<usize>) -> Self {
        Self {
            indices,
        }
    }
    pub fn len(&self) -> usize {
        self.indices.len()
    }
}


pub trait CollisionHandler {
    fn handle_collision<S>(&self, state: &mut S, idx1: usize, idx2: usize) 
    where S: State;
}

pub struct SimpleCollisionSolver {}
impl SimpleCollisionSolver {
    pub fn new() -> Self {
        Self {}
    }
}
impl CollisionHandler for SimpleCollisionSolver {
    fn handle_collision<S>(&self, state: &mut S, idx_i: usize, idx_j: usize) 
    where S: State
    {
        let bodies = state.get_bodies_mut();
        let body_i = &bodies[idx_i];
        let body_j = &bodies[idx_j];
        let collision_axis = body_i.position - body_j.position;
        let dist = collision_axis.magnitude();
        let correction_direction = collision_axis / dist;
        let collision_depth = body_i.radius + body_j.radius - dist;

        bodies[idx_i].position += 0.5*collision_depth*correction_direction;
        bodies[idx_j].position -= 0.5*collision_depth*correction_direction;
    }

}