use crate::engine::physics_engine::collision::CollisionBody;

use super::ConstraintResolver;

pub struct NoneConstraintResolver {}
#[allow(dead_code)]
impl NoneConstraintResolver {
    pub fn new() -> Self {
        Self {}
    }
}
#[allow(dead_code)]
impl ConstraintResolver for NoneConstraintResolver {
    fn resolve_vertical(&self, _diff: f32, _body: &mut CollisionBody) {}
    fn resolve_horizontal(&self, _diff: f32, _body: &mut CollisionBody) {}
}