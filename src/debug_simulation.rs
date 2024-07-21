use cgmath::{Vector3, Zero};

use crate::engine::{init_utils::create_grid_positions, physics_engine::{collision::CollisionBody, constraint::{box_constraint::BoxConstraint, Constraint}, integrator::verlet::VerletIntegrator}, renderer_engine::{shapes::circle::Circle, vertex::Vertex}, Simulation};

pub struct DebugSimulation {
    dt: f32,
    num_instances: u32,
    integrator: VerletIntegrator,
    constraint: Box<dyn Constraint>,
    
    // Render data
    indices: Vec<u16>,
    vertices: Vec<Vertex>,
    num_indices: u32,
    colors: Vec<Vector3<f32>>,
}

impl DebugSimulation {}
impl Simulation for DebugSimulation {
    fn new() -> Self {
        let dt = 0.001;

        let velocity = Vector3::new(0.002, 0.002, 0.0);
        let prev_positions = create_grid_positions(2, 1, 0.75, None); 
        let position = vec![prev_positions[0] + velocity, prev_positions[1] + velocity];
        let acceleration = vec![Vector3::zero(), Vector3::zero()];
        let bodies = vec![
            CollisionBody::new(0, position[0], 0.25),
            CollisionBody::new(0, position[1], 0.25)];
        let num_instances = bodies.len() as u32;
        let integrator = VerletIntegrator::new(
            f32::MAX, prev_positions, acceleration, bodies);

        let constraint = Box::new(BoxConstraint::new());

        // Render data
        let indices = Circle::compute_indices();
        let vertices = Circle::compute_vertices([0.0,0.0,0.0,], 1.0);
        let num_indices = Circle::get_num_indices();
        let colors = vec![Vector3::new(255.0, 0.0,0.0), Vector3::new(0.0,255.0,0.0)];
    
        Self { 
            dt, integrator, constraint,
            num_instances, indices, vertices, num_indices, colors}
    }

    fn update(&mut self) {
        self.integrator.update(self.dt);
        let bodies = self.integrator.get_bodies_mut();

        for b in bodies.iter_mut() {
            self.constraint.apply_constraint(b);
        }
        
    }

    fn get_bodies(&self) -> &Vec<crate::engine::physics_engine::collision::CollisionBody> {
        &self.integrator.get_bodies()
    }

    fn get_vertices(&self) -> &Vec<crate::engine::renderer_engine::vertex::Vertex> {
       &self.vertices 
    }

    fn get_indices(&self) -> &Vec<u16> {
       &self.indices 
    }

    fn get_colors(&self) -> &Vec<cgmath::Vector3<f32>> {
        &self.colors
    }

    fn get_num_active_instances(&self) -> u32 {
        self.num_instances
    }

    fn get_target_num_instances(&self) -> u32 {
        self.num_instances
    }

    fn get_num_indices(&self) -> u32 {
        self.num_indices
    }

    fn log_performance(&mut self) {
    }
}