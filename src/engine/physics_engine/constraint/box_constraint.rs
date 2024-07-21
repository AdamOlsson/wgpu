use cgmath::Vector3;

use crate::engine::physics_engine::collision::CollisionBody;

use super::Constraint;

pub struct BoxConstraint {}
impl BoxConstraint {
    pub fn new() -> Self {
        Self {}
    }
}

// TODO: Improve the constraint collision resolution
impl Constraint for BoxConstraint {
    fn apply_constraint(&self, body: &mut CollisionBody) {
        let pos = &mut body.position;
        let radius = body.radius;
        let constraint_top_left = Vector3::new(-1.0, 1.0, 0.0);
        let constraint_bottom_right = Vector3::new(1.0, -1.0, 0.0);
        // Left side
        if pos.x - radius < constraint_top_left.x {
            let diff = pos.x - radius  - constraint_top_left.x;
            pos.x -= diff*2.0;
        }
        // Right side
        if pos.x + radius > constraint_bottom_right.x {
            let diff = pos.x + radius - constraint_bottom_right.x; 
            pos.x -= diff*2.0;
        }
        // Bottom side
        if pos.y - radius < constraint_bottom_right.y {
            let diff = pos.y - radius - constraint_bottom_right.y;
            pos.y -= diff*2.0;
        }
        // Top side
        if pos.y + radius > constraint_top_left.y {
            let diff = pos.y + radius - constraint_top_left.y;
            pos.y -= diff*2.0;
        }
    }
}