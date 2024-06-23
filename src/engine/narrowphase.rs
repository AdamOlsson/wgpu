use cgmath::MetricSpace;

use super::collision::{CollisionBody, CollisionCandidates, CollisionHandler};

pub trait NarrowPhase {
    fn collision_detection<H>(
        &self, 
        bodies: &mut Vec<CollisionBody>,
        candidates: &CollisionCandidates, 
        handler: &H,
    ) 
    where
        H: CollisionHandler;
}

pub struct Naive {
}

impl Naive {
    pub fn new() -> Self {
        Self {}
    }
}

impl NarrowPhase for Naive {

    fn collision_detection<H>(
        &self,
        bodies: &mut Vec<CollisionBody>,
        candidates: &CollisionCandidates,
        solver: &H
    )
    where
        H: CollisionHandler,
    {
        let num_candidates = candidates.len();

        if num_candidates <= 1 {
            return;
        }

        for i in 0..num_candidates as usize {
            for j in 0..num_candidates as usize {
                if i == j {
                    continue;
                }
                let idx_i = candidates.indices[i];
                let idx_j = candidates.indices[j];
                let body_i = &bodies[idx_i];
                let body_j = &bodies[idx_j];

                let dist = body_i.position.distance(body_j.position);
                if dist == 0.0 {
                    panic!("Collision axis has zero length.");
                }
                if dist < (body_i.radius + body_j.radius) {
                    solver.handle_collision(bodies, idx_i, idx_j);
                }
            }
        } 
    }
}