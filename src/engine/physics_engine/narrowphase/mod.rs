use super::collision::{CollisionBody, CollisionCandidates};


pub mod naive;
pub trait NarrowPhase {
    fn collision_detection(
        &self, 
        bodies: &mut Vec<CollisionBody>,
        candidates: &CollisionCandidates, 
    );
}
