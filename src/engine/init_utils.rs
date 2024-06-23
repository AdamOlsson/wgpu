// pub(crate) fn generate_initial_positions_square_grid(num_instances: u32, radius: f32) -> [Vector3<f32>; MAX_INSTANCES]{
//     let grid_side = f32::round(f32::sqrt(num_instances as f32) + 1.0);
//     if grid_side * radius >= 2.0 {
//         panic!("Radius too large for number of instances.");
//     }

//     let delta = 2.0 / grid_side;
//     let mut positions = [Vector3::new(0.0,0.0,0.0); MAX_INSTANCES];
//     for i in 0..grid_side as usize {
//         for j in 0..grid_side as usize {
//             positions[i * grid_side as usize + j] = Vector3::new(
//                 delta * i as f32 - 1.0 + delta / 2.0,
//                 delta * j as f32 - 1.0 + delta / 2.0,
//                 0.0,
//             );
//         }
//     }
//     positions
// }

use cgmath::Vector3;
use rand::Rng;

pub(crate) fn generate_random_colors(n: u32) -> Vec<Vector3<f32>> {
    let mut colors = vec![];
    let mut rng = rand::thread_rng();
    let min = 0.2;
    let max = 1.0;
    for _ in 0..n {
        let c = Vector3::new(rng.gen_range(min..max), rng.gen_range(min..max), rng.gen_range(min..max));
        colors.push(c);
    }
    colors
}

// pub(crate) fn generate_random_velocities() -> [Vector3<f32>; MAX_INSTANCES] {
//     let mut velocities = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
//     let mut rng = rand::thread_rng();
//     for i in 0..MAX_INSTANCES {
//         velocities[i] = Vector3::new(rng.gen_range(-0.01..=0.01), rng.gen_range(-0.01..=0.01), 0.0);
//     }
//     velocities
// }