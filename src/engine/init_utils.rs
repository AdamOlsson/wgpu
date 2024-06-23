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

pub(crate) fn create_grid_positions(num_rows: u32, num_cols: u32, spacing: f32) -> Vec<Vector3<f32>> {
    let mut positions = Vec::new();
    let mut rng = rand::thread_rng();
    for i in 0..num_rows {
        for j in 0..num_cols {
            let mut x = (i as f32 - num_rows as f32 / 2.0) * spacing;
            let y = (j as f32 - num_cols as f32 / 2.0) * spacing;
            x += rng.gen_range(-0.01..0.01);
            positions.push(Vector3::new(x, y, 0.0));
        }
    }
    positions
}

pub(crate) fn generate_random_radii(num_instances: u32, base_radius: f32, variance: f32) -> Vec<f32> {
    if variance == 0.0 {
        return vec![base_radius; num_instances as usize];
    }
    let mut rng = rand::thread_rng();
    let mut radii = Vec::new();
    for _ in 0..num_instances {
        let radius = base_radius + rng.gen_range(-variance.abs()..variance.abs());
        radii.push(radius);
    }
    return radii;
}
// pub(crate) fn generate_random_velocities() -> [Vector3<f32>; MAX_INSTANCES] {
//     let mut velocities = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
//     let mut rng = rand::thread_rng();
//     for i in 0..MAX_INSTANCES {
//         velocities[i] = Vector3::new(rng.gen_range(-0.01..=0.01), rng.gen_range(-0.01..=0.01), 0.0);
//     }
//     velocities
// }