use crate::engine::physics_engine::collision::CollisionBody;

pub mod elastic;
pub mod none;

#[allow(unused_variables)]
pub trait ConstraintResolver {
    fn resolve_vertical(&self, diff: f32, body: &mut CollisionBody) {}
    fn resolve_horizontal(&self, diff: f32, body: &mut CollisionBody) {}
}
