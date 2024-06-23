use cgmath::Vector3;

use super::{naive::naive_solver, util::{assign_object_to_cell, get_local_cell_ids}};


pub(crate) fn spatial_subdivision_solver(
    positions: &mut [Vector3<f32>], radius: &[f32]
) {
    if positions.len() == 0 {
        return;
    }
    // Create grid with largest side equal to the largest diameter of the circles
    let cell_size = radius.iter().fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
    let grid_width = (2.0/cell_size).ceil() as u32;

    let cells = assign_object_to_cell(positions, radius);

    // For each cell, compute collision between all circles in the current cell and
    // all surrounding cells. Skip over the outer most cells.
    for i in 1..(grid_width-1) {
        for j in 1..(grid_width-1){
            let center_cell = i*grid_width + j;
            let local_cell_ids = get_local_cell_ids(center_cell as u32, grid_width);
            
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
            naive_solver(&mut local_positions, &local_radius);
            // Update positions
            for k in 0..num_local_instances as usize {
                positions[local_object_ids[k] as usize] = local_positions[k];
            }
        }
    }
}