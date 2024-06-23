use super::{collision::{CollisionBody, CollisionCandidates}, State};

pub trait BroadPhase {
    fn collision_detection(&self, bodies: &Vec<CollisionBody>) -> Vec<CollisionCandidates>;
}

pub struct SpatialSubdivision {}

impl SpatialSubdivision {
    pub fn new() -> Self {
        Self {}
    }

    fn assign_object_to_cell(&self, bodies: &Vec<CollisionBody>, cell_size: f32, grid_width: u32) -> Vec<Vec<usize>> {
    
       // Assign each circle to a cell
        let mut cells: Vec<Vec<usize>> = vec![Vec::new(); (grid_width*grid_width) as usize];
        for (i, b) in bodies.iter().enumerate() {
            let pos = b.position;
            // Add 1.0 to offset all coordinates between 0.0 and 2.0
            let x = ((pos.x + 1.0)/cell_size) as u32;
            let y = ((pos.y + 1.0)/cell_size) as u32;
            let cell_index = (y*grid_width + x) as usize;
            cells[cell_index].push(i);
        }
        return cells;
    }

    fn get_local_cell_ids(&self, center_id: u32, grid_width: u32) -> [u32; 9] {
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

}

impl BroadPhase for SpatialSubdivision {

    fn collision_detection(&self, bodies: &Vec<CollisionBody>) -> Vec<CollisionCandidates>  {
        
        if bodies.len() == 0 {
            return vec![]; 
        }
        // Create grid with largest side equal to the largest diameter of the circles
        let cell_size = bodies.iter().fold(0.0, |acc, b| f32::max(acc, b.radius))*2.0;
        let grid_width = (2.0/cell_size).ceil() as u32;
    
        let cells = self.assign_object_to_cell(bodies, cell_size, grid_width);
    
        // For each cell, compute collision between all circles in the current cell and
        // all surrounding cells. Skip over the outer most cells.
        let mut all_candidates = vec![];
        for i in 1..(grid_width-1) {
            for j in 1..(grid_width-1){
                let center_cell = i*grid_width + j;
                let local_cell_ids = self.get_local_cell_ids(center_cell as u32, grid_width);
            
                let collision_candidates: Vec<usize> = local_cell_ids.iter()
                    .map(| cell_id | cells[*cell_id as usize].clone())
                    .flatten()
                    .collect();
                
                if collision_candidates.len() <= 1 {
                    continue;
                }
                
                all_candidates.push(CollisionCandidates::new(collision_candidates));
            }
        }
        return all_candidates; 
    }



}