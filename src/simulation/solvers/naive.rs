use cgmath::{InnerSpace, Vector3};

pub(crate) fn naive_solver(
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