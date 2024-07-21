use cgmath::Vector3;
use crate::engine::physics_engine::collision::CollisionBody;
use super::{resolver::ConstraintResolver, Constraint};


pub struct BoxConstraint {
    resolver: Box<dyn ConstraintResolver> 
}
impl BoxConstraint {
    pub fn new<T: ConstraintResolver + 'static>(resolver: T) -> Self {
        Self {resolver: Box::new(resolver)}
    }
}

impl Constraint for BoxConstraint {
    fn apply_constraint(&self, body: &mut CollisionBody) {
        let constraint_top_left = Vector3::new(-1.0, 1.0, 0.0);
        let constraint_bottom_right = Vector3::new(1.0, -1.0, 0.0);
        
        // Left side
        if body.position.x - body.radius < constraint_top_left.x {
            let diff = body.position.x - body.radius  - constraint_top_left.x;
            self.resolver.resolve_horizontal(diff, body);
        }
        // Right side
        if body.position.x + body.radius > constraint_bottom_right.x {
            let diff = body.position.x + body.radius - constraint_bottom_right.x; 
            self.resolver.resolve_horizontal(diff, body);
        }
        // Bottom side
        if body.position.y - body.radius < constraint_bottom_right.y {
            let diff = body.position.y - body.radius - constraint_bottom_right.y;
            self.resolver.resolve_vertical(diff, body);
        }
        // Top side
        if body.position.y + body.radius > constraint_top_left.y {
            let diff = body.position.y + body.radius - constraint_top_left.y;
            self.resolver.resolve_vertical(diff, body);
        }
    }
}