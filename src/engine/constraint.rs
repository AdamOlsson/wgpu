use cgmath::{InnerSpace, Vector3};

use super::collision::CollisionBody;


pub trait Constraint {
    fn new() -> Self; 
    fn apply_constraint(&self, body: &mut CollisionBody);
}

pub struct BoxConstraint {}
impl Constraint for BoxConstraint {
    fn new() -> Self {
        Self {}
    }

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

pub struct CircleConstraint {}
impl Constraint for CircleConstraint {
    fn new() -> Self {
        Self {}
    }

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