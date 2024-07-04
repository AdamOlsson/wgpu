use std::{collections::HashSet, hash::Hasher, hash::Hash};

use cgmath::MetricSpace;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

use super::collision::{CollisionBody, CollisionCandidates};

pub trait BroadPhase {
    fn collision_detection(&self, bodies: &Vec<CollisionBody>) -> Vec<CollisionCandidates>;
}



/* ################# BLOCK MAP ################# */
pub struct BlockMap {}

impl BlockMap {
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

impl BroadPhase for BlockMap {

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


/* ############## SWEEP AND PRUNE ############## */
struct Edge<'a> {
    object: &'a CollisionBody,
    x_value: f32,
    is_left: bool,
}

impl <'a> PartialEq for Edge<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.object.id == other.object.id && self.is_left == other.is_left
    }
    
    fn ne(&self, other: &Self) -> bool {
        !self.eq(other)
    }
}

impl <'a> Eq for Edge<'a> {}

impl <'a> Hash for Edge<'a> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.object.id.hash(state);
        self.is_left.hash(state);
    }
}


impl<'a> Edge<'a> {
    fn new(object: &'a CollisionBody, value: f32, is_lower: bool) -> Self {
        Self { object, x_value: value, is_left: is_lower }
    }
}
    
pub struct SweepAndPrune {}
impl SweepAndPrune {
    pub fn new() -> Self {
        panic!("SweepAndPrune does not work");
        Self {}
    }
    fn sort_ascending(edges: &mut Vec<Edge>) {
        edges.sort_by(| a, b | a.x_value.total_cmp(&b.x_value))
    }

}

impl BroadPhase for SweepAndPrune {
    fn collision_detection(
        &self, bodies: &Vec<CollisionBody>
    ) -> Vec<CollisionCandidates> {
        
        // Project edges onto x-axis
        let mut x_axis: Vec<Edge> = bodies.par_iter()
            .map( 
                | body | -> Vec<Edge> {
                    vec![Edge::new(body, body.position.x-body.radius, true), Edge::new(body, body.position.x+body.radius, false)]
                })
            .flatten()
            .collect();

        Self::sort_ascending(&mut x_axis);

        
        let mut collision_candidates = vec![];
        let mut active_sweep: HashSet<&Edge> = HashSet::new();
        x_axis.iter().for_each(| e |
            if e.is_left {
                let candidates: Vec<usize> = active_sweep.par_iter().filter_map(| active_edge: &&Edge | {
                    let min_dist_sq = e.object.radius + active_edge.object.radius;
                    let dist_sq =  e.object.position.distance(active_edge.object.position);
                    if dist_sq < min_dist_sq {
                        return Some(e.object.id);
                    } else {
                        return None;
                    }
                }).collect();
                collision_candidates.push(CollisionCandidates::new(candidates));

                active_sweep.insert(e);
            } else {
                active_sweep.remove(e);
            });

        
        return collision_candidates;
    }
}