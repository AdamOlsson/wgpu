use core::panic;

use cgmath::{InnerSpace, Vector3};

use crate::engine::physics_engine::collision::CollisionBody;

use super::Constraint;


pub struct CircleConstraint {}
#[allow(unreachable_code, dead_code)]
impl CircleConstraint{
    fn new() -> Self {
        panic!("Circle constraint is deprecated");
        Self {}
    }
}

impl Constraint for CircleConstraint {
    fn apply_constraint(&self, body: &mut CollisionBody) {
        let constraint_center = Vector3::new(0.0,0.0,0.0);
        let constraint_radius = &0.95;
    
        let diff = body.position - constraint_center;
        let dist = diff.magnitude();
        if dist > (constraint_radius - body.radius) {
            let correction_direction = diff / dist;
            body.position = constraint_center + correction_direction*(constraint_radius - body.radius);
        }
    }
    
}
