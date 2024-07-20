use super::collision::CollisionBody;

pub mod circle_constraint;
pub mod box_constraint;

pub trait Constraint {
    fn new() -> Self; 
    fn apply_constraint(&self, body: &mut CollisionBody);
}