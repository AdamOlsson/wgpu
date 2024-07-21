use cgmath::{InnerSpace, Vector3};

use crate::engine::physics_engine::collision::CollisionBody;


pub struct VerletIntegrator {
    velocity_cap: f32,
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>,
    bodies: Vec<CollisionBody>,
}

impl VerletIntegrator {
    pub fn new(
        velocity_cap: f32, prev_positions: Vec<Vector3<f32>>,
        acceleration: Vec<Vector3<f32>>, bodies: Vec<CollisionBody>
    ) -> Self {
        Self { velocity_cap, prev_positions, acceleration, bodies}
    }

    pub fn update(&mut self, dt: f32) {
        for (i, b) in self.bodies.iter_mut().enumerate() {
            let mut velocity = b.position - self.prev_positions[i];
            let vel_magn = velocity.magnitude();
            if vel_magn > self.velocity_cap {
                velocity = velocity*(self.velocity_cap/vel_magn)
            }

            self.prev_positions[i] = b.position;
            b.position = b.position + velocity + self.acceleration[i] * dt*dt;   
        }
    }

    pub fn set_acceleration_y(&mut self, idx: usize, new: f32) {
        self.acceleration[idx].y = new;
    }

    pub fn get_bodies(&self) -> &Vec<CollisionBody> {
        &self.bodies
    }

    pub fn get_bodies_mut(&mut self) -> &mut Vec<CollisionBody> {
        &mut self.bodies
    }

    #[allow(dead_code)]
    pub fn update_subset(&mut self){
        todo!("Not yet implementd");
    }
}