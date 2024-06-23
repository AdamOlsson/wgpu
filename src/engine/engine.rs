use super::{broadphase::BroadPhase, collision::{CollisionCandidates, CollisionHandler}, constraint::Constraint, narrowphase::NarrowPhase, State};


// TODO: Apply strategy pattern
pub struct Engine {}
impl Engine {
    pub fn new() -> Self {
        Self {}
    }

    pub fn broad_phase_collision_detection<S, D>(state: &S, detector: &D) -> CollisionCandidates
    where
        S: State,
        D: BroadPhase,
    {
        //detector.collision_detection(state)
        CollisionCandidates::new(vec![])
    }

    pub fn narrow_phase_collision_detection<S, D, H>(
        state: &mut S,
        candidates: &CollisionCandidates,
        detector: &D,
        handler: &H,
    )
    where
        S: State,
        D: NarrowPhase,
        H: CollisionHandler
    {
        detector.collision_detection(state, candidates, handler);
    }


    pub fn constraint_handler<S, C>(state: &mut S, constraint: &C)
    where 
        S: State,
        C: Constraint,
    {
        let bodies = state.get_bodies();
        let num_instances = bodies.len(); // FIXME: Use states num_instances
        for i in 0..num_instances as usize {
            //constraint.apply_constraint(&mut bodies[i]);
        }
    }

}
