use cgmath::Vector3;
use crate::engine::physics_engine::collision::CollisionBody;
use super::{resolver::ConstraintResolver, Constraint};


pub struct BoxConstraint {
    resolver: Box<dyn ConstraintResolver>,
    top_left: Vector3<f32>,
    bottom_right: Vector3<f32>
}

impl BoxConstraint {
    pub fn new<T: ConstraintResolver + 'static>(resolver: T) -> Self {
        let top_left = Vector3::new(-1.0, 1.0, 0.0);
        let bottom_right = Vector3::new(1.0, -1.0, 0.0);
        Self {resolver: Box::new(resolver), top_left, bottom_right }
    }

    #[allow(dead_code)] 
    pub fn set_top_left(&mut self, top_left: Vector3<f32>) {
        self.top_left = top_left;
    }
    
    #[allow(dead_code)] 
    pub fn set_bottom_right(&mut self, bottom_right: Vector3<f32>) {
        self.bottom_right = bottom_right;
    }
}

impl Constraint for BoxConstraint {
    fn apply_constraint(&self, body: &mut CollisionBody) {
        // Left side
        if body.position.x - body.radius < self.top_left.x {
            let diff = body.position.x - body.radius  - self.top_left.x;
            self.resolver.resolve_horizontal(diff, body);
        }
        // Right side
        if body.position.x + body.radius > self.bottom_right.x {
            let diff = body.position.x + body.radius - self.bottom_right.x; 
            self.resolver.resolve_horizontal(diff, body);
        }
        // Bottom side
        if body.position.y - body.radius < self.bottom_right.y {
            let diff = body.position.y - body.radius - self.bottom_right.y;
            self.resolver.resolve_vertical(diff, body);
        }
        // Top side
        if body.position.y + body.radius > self.top_left.y {
            let diff = body.position.y + body.radius - self.top_left.y;
            self.resolver.resolve_vertical(diff, body);
        }
    }
}
