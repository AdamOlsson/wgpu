
use core::fmt;
use std::fmt::Display;

use cgmath::{InnerSpace, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct CollisionBody {
    pub velocity: Vector3<f32>,
    pub prev_position: Vector3<f32>,
    pub position: Vector3<f32>,
    pub radius: f32,
    pub id: usize
}

impl CollisionBody {
    pub fn new(
        id: usize, velocity: Vector3<f32>, prev_position: Vector3<f32>,
        position: Vector3<f32>, radius: f32
    ) -> Self {
        Self {
            velocity,
            prev_position,
            position,
            radius,
            id
        }
    }
}

impl Display for CollisionBody {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "id: {}, position: ({},{},{}), prev_pos: ({},{},{}), radius: {}, velocity: ({},{},{})",
            self.id,
            self.position.x, self.position.y, self.position.z,
            self.prev_position.x, self.prev_position.y, self.prev_position.z,
            self.radius,
            self.velocity.x, self.velocity.y, self.velocity.z)
    }
}

#[derive(Debug)]
pub struct CollisionCandidates {
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
    fn handle_collision(&self, bodies: &mut Vec<CollisionBody>, idx_i: usize, idx_j: usize);
}

pub struct SimpleCollisionSolver {}
impl SimpleCollisionSolver {
    pub fn new() -> Self {
        Self {}
    }
}

impl CollisionHandler for SimpleCollisionSolver {
    fn handle_collision(
        &self, bodies: &mut Vec<CollisionBody>, idx_i: usize, idx_j: usize) 
    {
        let body_i = &bodies[idx_i];
        let body_j = &bodies[idx_j];
        let collision_axis = body_i.position - body_j.position;
        let collision_normal = collision_axis.normalize();
        let dist = collision_axis.magnitude();
        let correction_direction = collision_axis / dist;
        let collision_depth = body_i.radius + body_j.radius - dist;

        bodies[idx_i].position += 0.5*collision_depth*correction_direction;
        bodies[idx_j].position -= 0.5*collision_depth*correction_direction;

        let p = bodies[idx_i].velocity.dot(collision_normal) - bodies[idx_j].velocity.dot(collision_normal);
        bodies[idx_i].velocity = bodies[idx_i].velocity - p * collision_normal; 
        bodies[idx_j].velocity = bodies[idx_j].velocity + p * collision_normal;

        bodies[idx_i].prev_position = bodies[idx_i].position - bodies[idx_i].velocity;
        bodies[idx_j].prev_position = bodies[idx_j].position - bodies[idx_j].velocity;
    }
}
