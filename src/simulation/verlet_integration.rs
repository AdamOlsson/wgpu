
use std::{sync::{Arc, Mutex}, thread};
use cgmath::{InnerSpace, Vector3};

use super::solvers;


type ConstraintFn = fn(&mut Vector3<f32>, &f32);
type SolverFn = fn(&mut [Vector3<f32>], &[f32]);

#[allow(dead_code)]
#[derive(Debug)]
pub enum SolverType {
    Naive,
    SpatialSubdivision,
    ParallelNaive,
    //ParallelSpatialSubdivision,
}

pub struct VerletIntegration
{
    positions: Vec<Vector3<f32>>,
    prev_positions: Vec<Vector3<f32>>,
    acceleration: Vec<Vector3<f32>>, 
    radii: Vec<f32>,

    constraint_fn:  ConstraintFn,
    solver_fn: SolverFn,
    num_solver_steps : u32,

    num_instances: u32,
}

impl VerletIntegration { 
    pub fn new() -> Self {
        
        let prev_positions = vec![];
        let positions = vec![];
        let acceleration = vec![];
        let radii = vec![];

        let num_instances = acceleration.len() as u32;

        let constraint_fn = |_: &mut Vector3<f32>, _: &f32 | {
            panic!("Constraint function not set");
        };

        let solver_fn = |_: &mut [Vector3<f32>], _: &[f32]| {
            panic!("Solver function not set");
        };
        let num_solver_steps = 1;

        Self {
            positions,
            prev_positions,
            acceleration,
            radii,
            num_instances,
            constraint_fn,
            solver_fn,
            num_solver_steps,
        }
    }

    #[allow(dead_code)]
    pub fn spawn_one_instance(&mut self) {
        self.num_instances += 1;
    }

    #[allow(dead_code)]
    pub fn spawn_all_instances(&mut self) {
        self.num_instances = self.acceleration.len() as u32;
    }

    pub fn use_constraint(&mut self, f: ConstraintFn) {
        self.constraint_fn = f;
    }

    pub fn use_solver(&mut self, t: SolverType) {
        let f = match t {
            SolverType::Naive => solvers::naive::naive_solver,
            SolverType::SpatialSubdivision => solvers::spatial_subdivision::spatial_subdivision_solver, 
            SolverType::ParallelNaive => solvers::par_naive::par_naive_solver,

            //SolverType::ParallelSpatialSubdivision => Self::par_spatial_subdivision,
        };
        self.solver_fn = f;
    }

    pub fn set_num_solver_steps(&mut self, num_solver_steps: u32) {
        self.num_solver_steps = num_solver_steps;
    }

    pub fn set_radii(&mut self, radii: Vec<f32>) {
        self.radii = radii;
    }

    pub fn set_positions(&mut self, positions: Vec<Vector3<f32>>){
        self.positions = positions;
    }

    pub fn set_prev_positions(&mut self, prev_positions: Vec<Vector3<f32>>) {
        self.prev_positions = prev_positions;
    } 

    pub fn set_acceleration(&mut self, acceleration: Vec<Vector3<f32>>) {
        self.acceleration = acceleration;
    }

    pub fn get_positions(&self) -> &Vec<Vector3<f32>> {
        &self.positions
    }

    pub fn get_radii(&self) -> &Vec<f32> {
        &self.radii
    }

    pub fn update(&mut self, dt: f32) {
        // Update positions
        for i in 0..self.num_instances as usize {
            let velocity = self.positions[i] - self.prev_positions[i];
            self.prev_positions[i] = self.positions[i];
            self.positions[i] = self.positions[i] + velocity + self.acceleration[i] * dt*dt;   
        }

        // Constraints
        for i in 0..self.num_instances  as usize {
            (self.constraint_fn)(&mut self.positions[i], &self.radii[i]);
        }

        // Solve collisions
        let pos_slice = &mut self.positions[0..self.num_instances as usize];
        let rad_slice = &mut self.radii[0..self.num_instances as usize];
        for _ in 0..self.num_solver_steps {
            (self.solver_fn)(pos_slice, rad_slice);
        }
    }


    fn get_local_cell_ids(center_id: u32, grid_width: u32) -> [u32; 9] {
        let top_left = center_id - grid_width - 1;
        let top_center = center_id - grid_width;
        let top_right = center_id - grid_width + 1;
        let center_left = center_id - 1;
        let center_right = center_id + 1;
        let bottom_left = center_id + grid_width - 1;
        let bottom_center = center_id + grid_width;
        let bottom_right = center_id + grid_width + 1;
        return [
            top_left, top_center, top_right,
            center_left, center_id, center_right,
            bottom_left, bottom_center, bottom_right
        ];
    }

    fn assign_object_to_cell(positions: &[Vector3<f32>], radius: &[f32]) -> Vec<Vec<usize>> {
        let cell_size = radius.iter().fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
        let grid_width = (2.0/cell_size).ceil() as u32;

       // Assign each circle to a cell
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); (grid_width*grid_width) as usize];
        for (i, pos) in positions.iter().enumerate() {
            // Add 1.0 to offset all coordinates between 0.0 and 2.0
            let x = ((pos.x + 1.0)/cell_size) as u32;
            let y = ((pos.y + 1.0)/cell_size) as u32;
            let cell_index = (y*grid_width + x) as usize;
            cells[cell_index].push(i);
        }
        return cells;
    }

    
}


#[allow(dead_code)]
pub fn circle_constraint(
    pos: &mut Vector3<f32>, radius: &f32
)  {
    let constraint_center = Vector3::new(0.0,0.0,0.0);
    let constraint_radius = &0.95;

    let diff = *pos - constraint_center;
    let dist = diff.magnitude();
    if dist > (constraint_radius - radius) {
        let correction_direction = diff / dist;
        *pos = constraint_center + correction_direction*(constraint_radius - radius);
    }
}

#[allow(dead_code)]
pub fn box_constraint(
    pos: &mut Vector3<f32>, radius: &f32
) { 
    let constraint_top_left = Vector3::new(-1.0, 1.0, 0.0);
    let constraint_bottom_right = Vector3::new(1.0, -1.0, 0.0);
    // Left side
    if pos.x - radius < constraint_top_left.x {
        let diff = pos.x - radius  - constraint_top_left.x;
        pos.x -= diff*2.0;
    }
    // Right side
    if pos.x + radius > constraint_bottom_right.x {
        let diff = pos.x + radius - constraint_bottom_right.x; 
        pos.x -= diff*2.0;
    }
    // Bottom side
    if pos.y - radius < constraint_bottom_right.y {
        let diff = pos.y - radius - constraint_bottom_right.y;
        pos.y -= diff*2.0;
    }
    // Top side
    if pos.y + radius > constraint_top_left.y {
        let diff = pos.y + radius - constraint_top_left.y;
        pos.y -= diff*2.0;
    }
}