pub mod engine;
pub mod simulation;
pub mod collision;
pub mod state;
pub mod constraint;
pub mod broadphase;
pub mod narrowphase;
pub mod init_utils;

use collision::CollisionBody;

pub trait State {
    fn get_bodies(&self) -> &Vec<CollisionBody>;
    fn get_bodies_mut(&mut self) -> &mut Vec<CollisionBody>;
}