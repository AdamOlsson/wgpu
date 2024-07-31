use cgmath::MetricSpace;
use crate::engine::physics_engine::collision::{CollisionBody, CollisionCandidates, CollisionHandler};
use super::NarrowPhase;

pub struct Naive {
    solver: Box<dyn CollisionHandler + 'static>
}

impl Naive {
    pub fn new<H>(solver: H) -> Self
    where
        H: CollisionHandler + 'static,
    {
        let s: Box<dyn CollisionHandler> = Box::new(solver);
        Self { solver: s }
    }
}

impl NarrowPhase for Naive {
    fn collision_detection(
        &self,
        bodies: &mut Vec<CollisionBody>,
        candidates: &CollisionCandidates,
    ) {
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
                    self.solver.handle_collision(bodies, idx_i, idx_j);
                }
            }
        } 
    }
}
