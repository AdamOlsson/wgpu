use std::{sync::{Arc, Mutex}, thread};

use cgmath::Vector3;

use super::{naive::naive_solver, util::assign_object_to_cell};

pub(crate) fn par_naive_solver(
    positions: &mut [Vector3<f32>], radius: &[f32]
){
    if positions.len() <= 1 {
        return;
    }
    // Create grid with largest side equal to the largest diameter of the circles
    let cell_size = radius.iter().fold(0.0, |acc, &r| f32::max(acc, r))*2.0;
    let grid_width = (2.0/cell_size).ceil() as u32;

    let cells = assign_object_to_cell(positions, radius);

    run_pass(1, positions, radius, &cells, grid_width);
    run_pass(2, positions, radius, &cells, grid_width);
    run_pass(3, positions, radius, &cells, grid_width);
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
        let thread_local_cell_ids = get_thread_context_cell_ids(column_id, grid_width);
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
                naive_solver(&mut thread_local_positions[..], &thread_local_radius);
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