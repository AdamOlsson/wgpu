use super::collision::CollisionBody;

pub mod circle_constraint;
pub mod box_constraint;

pub trait Constraint {
    fn apply_constraint(&self, body: &mut CollisionBody);
}