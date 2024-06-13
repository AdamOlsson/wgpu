
use std::{sync::{Arc, Mutex}, thread};
use cgmath::{InnerSpace, Vector3};

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
    pub positions: Vec<Vector3<f32>>,
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
            SolverType::Naive => Self::naive_collision_detection_and_resolution,
            SolverType::SpatialSubdivision => Self::spatial_subdivision,
            SolverType::ParallelNaive => Self::par_naive_collision_detection_and_resolution,
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

    fn naive_collision_detection_and_resolution(
        positions: &mut [Vector3<f32>], radius: &[f32]
    ){
        let num_instances = positions.len() as usize;
        for i in 0..num_instances as usize {
            let pos_a = positions[i];
            for j in (i+1)..num_instances as usize {
                let pos_b = positions[j];
                let collision_axis = pos_a - pos_b;
                let dist = collision_axis.magnitude();
                if dist == 0.0 {
                    panic!("Collision axis has zero length");
                }
                if dist < radius[i] + radius[j] {
                    let correction_direction = collision_axis / dist;
                    let collision_depth = radius[i] + radius[j] - dist;
                    positions[i] += 0.5*collision_depth*correction_direction;
                    positions[j] -= 0.5*collision_depth*correction_direction;
                }
            }
        }
    }

    #[allow(dead_code)]
    fn spatial_subdivision(
        positions: &mut [Vector3<f32>], radius: &[f32]
    ) {
        if positions.len() == 0 {
            return;
        }
        // Create grid with largest side equal to the largest diameter of the circles
        let cell_size = radius.iter().fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
        let grid_width = (2.0/cell_size).ceil() as u32;

        let cells = Self::assign_object_to_cell(positions, radius);

        // For each cell, compute collision between all circles in the current cell and
        // all surrounding cells. Skip over the outer most cells.
        for i in 1..(grid_width-1) {
            for j in 1..(grid_width-1){
                let center_cell = i*grid_width + j;
                let local_cell_ids = Self::get_local_cell_ids(center_cell as u32, grid_width);
                
                // Collect local positions and radii
                let mut local_positions: Vec<Vector3<f32>> = Vec::new();
                let mut local_radius: Vec<f32> = Vec::new();
                let mut local_object_ids: Vec<usize> = Vec::new();
                for cell_id in local_cell_ids.iter() {
                    for object_id in cells[*cell_id as usize].iter() {
                        local_positions.push(positions[*object_id].clone());
                        local_radius.push(radius[*object_id].clone());
                        local_object_ids.push(*object_id);
                    }
                }

                let num_local_instances = local_positions.len() as u32;
                if num_local_instances <= 1 {
                    continue;
                }
                Self::naive_collision_detection_and_resolution(&mut local_positions, &local_radius);
                // Update positions
                for k in 0..num_local_instances as usize {
                    positions[local_object_ids[k] as usize] = local_positions[k];
                }
            }
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

    fn assign_object_to_cell(positions: &mut [Vector3<f32>], radius: &[f32]) -> Vec<Vec<usize>> {
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

    fn par_naive_collision_detection_and_resolution(
        positions: &mut [Vector3<f32>], radius: &[f32]
    ){
        if positions.len() <= 1 {
            return;
        }
        // Create grid with largest side equal to the largest diameter of the circles
        let cell_size = radius.iter().fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
        let grid_width = (2.0/cell_size).ceil() as u32;

        let cells = Self::assign_object_to_cell(positions, radius);

        Self::run_pass(1, positions, radius, &cells, grid_width);
        Self::run_pass(2, positions, radius, &cells, grid_width);
        Self::run_pass(3, positions, radius, &cells, grid_width);
    }

    fn get_thread_context_cell_ids(center_column_id: u32, grid_width: u32) -> Vec<u32> {
        let mut cell_ids = Vec::new();
        let left_col_ids= ((center_column_id-1)..(grid_width*grid_width)).step_by(grid_width as usize);
        let center_col_ids = ((center_column_id)..(grid_width*grid_width)).step_by(grid_width as usize);
        let right_col_ids = ((center_column_id+1)..(grid_width*grid_width)).step_by(grid_width as usize);
        cell_ids.extend(left_col_ids);
        cell_ids.extend(center_col_ids);
        cell_ids.extend(right_col_ids);
        return cell_ids;
    }

    fn run_pass(
        pass_num: u32, positions: &mut [Vector3<f32>], radius: &[f32],
        cells: &Vec<Vec<usize>>, grid_width: u32
    ) {
        let mut localized_positions = Vec::new();
        let mut localized_radius = Vec::new();
        let mut localized_object_ids = Vec::new();
        for column_id in (pass_num..grid_width-1).step_by(3) {
            let thread_local_cell_ids = Self::get_thread_context_cell_ids(column_id, grid_width);
            // Assign cell ids to columns 
            let mut thread_local_positions = Vec::new();
            let mut thread_local_radius = Vec::new();
            let mut thread_local_object_ids = Vec::new();
            for cell_id in thread_local_cell_ids.iter() {
                for object_id in cells[*cell_id as usize].iter() {
                    thread_local_positions.push(positions[*object_id]);
                    thread_local_radius.push(radius[*object_id]);
                    thread_local_object_ids.push(*object_id);
                }
            }
            localized_positions.push(thread_local_positions);
            localized_radius.push(thread_local_radius);
            localized_object_ids.push(thread_local_object_ids);
            
        }
        // Launch threads
        let output = Arc::new(Mutex::new(Vec::new()));
        thread::scope(|scope| {
            for (thread_id, thread_local_positions) in localized_positions.iter_mut().enumerate() {
                let thread_local_radius = &localized_radius[thread_id];
                let global_output = Arc::clone(&output);
                scope.spawn(move || {
                    //Self::spatial_subdivision(&mut thread_local_positions[..], &thread_local_radius);
                    Self::naive_collision_detection_and_resolution(&mut thread_local_positions[..], &thread_local_radius);
                    global_output.lock().unwrap().push((thread_id, thread_local_positions));
                });
            }
        });
        // Write back results
        let inner_output = Arc::try_unwrap(output).unwrap().into_inner().unwrap();
        for (thread_id, thread_local_positions) in inner_output {
            let thread_local_object_ids = &localized_object_ids[thread_id];
            let num_local_instances = thread_local_positions.len();
            for k in 0..num_local_instances as usize {
                let object_id = thread_local_object_ids[k] as usize;
                positions[object_id] = thread_local_positions[k];
            }
        }
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