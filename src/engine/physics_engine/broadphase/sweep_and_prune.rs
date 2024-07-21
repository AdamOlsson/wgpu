
use std::{collections::HashSet, hash::Hasher, hash::Hash};
use crate::engine::physics_engine::collision::{CollisionBody, CollisionCandidates};
use cgmath::MetricSpace;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use super::BroadPhase;



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
#[allow(dead_code)]
impl SweepAndPrune {
    pub fn new() -> Self {
        panic!("SweepAndPrune does not work");
        //Self {}
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