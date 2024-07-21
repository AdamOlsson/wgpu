use crate::engine::physics_engine::collision::CollisionBody;

use super::ConstraintResolver;


pub struct ElasticConstraintResolver {}
impl ElasticConstraintResolver {
    pub fn new() -> Self {
        Self {}
    }
}

impl ConstraintResolver for ElasticConstraintResolver {
    fn resolve_vertical(&self, diff: f32, body: &mut CollisionBody) {
        body.position.y -= diff;
        body.velocity.y *= -1.0;
        body.prev_position = body.position - body.velocity;
    }

    fn resolve_horizontal(&self, diff: f32, body: &mut CollisionBody) {
        body.position.x -= diff;
        body.velocity.x *= -1.0;
        body.prev_position = body.position - body.velocity; 
    }
}