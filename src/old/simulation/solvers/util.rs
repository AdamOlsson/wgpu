use cgmath::Vector3;

pub(crate) fn assign_object_to_cell(positions: &[Vector3<f32>], radius: &[f32]) -> Vec<Vec<usize>> {
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

pub(crate) fn get_local_cell_ids(center_id: u32, grid_width: u32) -> [u32; 9] {
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