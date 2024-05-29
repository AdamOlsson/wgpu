use core::panic;
use cgmath::{InnerSpace, Vector2, Vector3};

/* 
Cell ID layout:
-------
|0|1|2|
|3|4|5|
|6|7|8|
-------

Cell type layout:
-------
|0|1|0|
|2|3|2|
|0|1|0|
*/

const SPATIAL_SUBDIVISION_2D_CELL_OFFSET: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Object {
    id: u32,
    control_bits: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CollisionCell {
    offset: usize,
    num_objects: u32,
}

pub struct SpatialSubdivision2D {
    cell_size: f32,
    num_cells: u32,
}

impl SpatialSubdivision2D {
    pub fn new(cell_size: f32) -> Self {
        // We use ceil to handle the scenario where the cell size is not a multiple of 2.0
        let num_cells = (2.0 / cell_size).ceil();
        Self {
            cell_size,
            num_cells: num_cells as u32,
        }
    }
    /// Given a point, return the cell id that the point belongs to.
    /// 
    /// Assumes that the point is in the range of 0.0 to 2.0.
    ///
    /// Returns the cell id.
    fn point_to_cell_id(&self, point: Vector2<f32>) -> u32 {
        let mut x = (point.x / self.cell_size).floor();
        let mut y = ((2.0 - point.y) / self.cell_size).floor();

        // If the point is on the top or right, we need to adjust the cell id
        if x >= self.num_cells as f32 {
            x -= 1.0;
        }
        if y >= self.num_cells as f32 {
            y -= 1.0;
        }

        y as u32 * self.num_cells + x as u32
    }

    fn get_cell_id_center(cell_id: u32, cell_size: f32, num_cells: u32) -> Vector2<f32> {
        let x = (cell_id % num_cells) as f32 * cell_size + cell_size / 2.0;
        let y =  2.0 - (cell_id / num_cells) as f32 * cell_size - cell_size / 2.0;
        Vector2::new(x, y)
    }

    /// Given a cell return all neighbor cell ids.
    fn get_neighbor_cell_ids(cell_id: i32, num_cells: i32) -> [u32;8] {
        
        let top_cell_id: i32  = cell_id - num_cells;
        let bottom_cell_id: i32  = cell_id + num_cells;

        let left_cell_id: i32;
        let top_left_cell_id: i32;
        let bottom_left_cell_id: i32;
        if cell_id % num_cells < 1 {
            // Left edge, set to arbitrary negative value
            left_cell_id = -1;
            top_left_cell_id = -1;
            bottom_left_cell_id = -1;
        } else {
            left_cell_id = cell_id - 1;
            top_left_cell_id = cell_id - num_cells - 1;
            bottom_left_cell_id = cell_id + num_cells - 1;
        }

        let right_cell_id: i32;
        let top_right_cell_id: i32;
        let bottom_right_cell_id: i32;
        if cell_id % num_cells >= num_cells - 1 {
            // Right edge, set to arbitrary negative value
            right_cell_id = -1;
            top_right_cell_id = -1;
            bottom_right_cell_id = -1;
        } else {
            right_cell_id = cell_id + 1;
            top_right_cell_id = cell_id - num_cells + 1;
            bottom_right_cell_id = cell_id + num_cells + 1;
        }

        let neighbors_i32 = [
            top_left_cell_id, top_cell_id, top_right_cell_id,
            left_cell_id, right_cell_id,
            bottom_left_cell_id, bottom_cell_id, bottom_right_cell_id
        ];

        let mut neighbors: [u32;8] = [u32::MAX;8];
        for i in 0..8 {
            if 0 <= neighbors_i32[i] && neighbors_i32[i] < num_cells.pow(2){
                neighbors[i] = neighbors_i32[i] as u32;
            }
        }
        return neighbors
    }

    /// Compute the cell ids that are covered by the bounding sphere.
    /// 
    /// Given a bounding sphere, compute the cell ids that are covered by the bounding box.
    /// Assumes that the given bounding box have points between 0.0 and 2.0. 
    /// 
    /// Returns a list of cell ids that are covered by the bounding box where the first
    /// cell id is the home cell id.
    fn compute_phantom_cell_ids(&self, bounding_sphere: Vector3<f32>, cell_size: f32, num_cells: u32) -> Vec<u32> {        
        let sphere_center = Vector2::new(bounding_sphere.x, bounding_sphere.y);
        let sphere_radius = bounding_sphere.z;
        let clamped_sphere_center = Vector2::new(
            sphere_center.x.min(1.999999).max(0.0),
            sphere_center.y.min(1.999999).max(0.0)
        );
        let home_cell_id = self.point_to_cell_id(clamped_sphere_center);

        // The bounding sphere can only overlap 4 of these cells
        let neighbors_ids = SpatialSubdivision2D::get_neighbor_cell_ids(home_cell_id as i32, num_cells as i32);

        let mut phantom_cell_ids: Vec<u32> = Vec::new();
        phantom_cell_ids.push(home_cell_id);
        for neighbor_id in neighbors_ids.iter() {
            if neighbor_id == &u32::MAX {
                continue;
            }
            let neighbor_center = SpatialSubdivision2D::get_cell_id_center(*neighbor_id, cell_size, num_cells);

            let sphere_edge_point = (neighbor_center - sphere_center).normalize() * sphere_radius + sphere_center;
            
            let clamped_sphere_edge_point = Vector2::new(
                sphere_edge_point.x.min(1.999999).max(0.0),
                sphere_edge_point.y.min(1.999999).max(0.0)
            );
            
            let edge_cell = self.point_to_cell_id(clamped_sphere_edge_point);
            if neighbor_id == &edge_cell {
                phantom_cell_ids.push(*neighbor_id);
            }
        }
        return phantom_cell_ids;
    }

    /// Given a cell id, return the cell type.
    /// 
    /// To prevent updating the same object twice in a parallell scenario, we need to
    /// prevent an object to be processed by two threads in the same pass. This is 
    /// done by creating a repeating 2x2 pattern of cell types over the grid. These
    /// cell types are in a 2D scenario 0, 1, 2, 3 (top left, top right, bottom left,
    /// bottom right respective). Objects that are in the same cell type can be
    /// processed in parallell due to the assumption that each cell in the grid is
    /// equal to or larger than the size of the largest bounding box.
    /// 
    /// Returns the cell type.
    fn cell_id_to_cell_type(cell_id: u32, num_cells: u32) -> u8 {
        let even_col = (cell_id % num_cells) % 2 == 0;
        let even_row = (cell_id / num_cells) % 2 == 0;
        if even_row && even_col {
            return 0;
        } else if even_row && !even_col {
            return 1;
        } else if !even_row && even_col {
            return 2;
        } else {
            return 3;
        }
    }

    fn _compute_home_cell_id(&self, bounding_sphere: Vector3<f32>) -> u32 {
        let center_x = bounding_sphere.x + (bounding_sphere.z / 2.0);
        let center_y = bounding_sphere.y - (bounding_sphere.z / 2.0);
        return self.point_to_cell_id(Vector2::new(center_x, center_y));
    }

    fn construct_arrays(&self, bounding_spheres: &Vec<Vector3<f32>>, cells: &mut Vec<u32>, objects: &mut Vec<Object>) {
        for (i, bbox) in bounding_spheres.iter().enumerate() {
            // Offset the bbox to have 0.0 and bottom left corner
            let offset = i * SPATIAL_SUBDIVISION_2D_CELL_OFFSET;
            let bbox_offset = bbox + Vector3::new(1.0, 1.0, 0.0);
            
            
            let phantom_cell_ids = self.compute_phantom_cell_ids(bbox_offset, self.cell_size, self.num_cells);
            let home_cell_id = phantom_cell_ids[0];
            
            let mut phantom_cell_bits: u8 = 0;
            for cell_id in phantom_cell_ids.iter() {
                phantom_cell_bits |= 1 << SpatialSubdivision2D::cell_id_to_cell_type(cell_id.clone(), self.num_cells);
            }
                        
            let home_cell_type = SpatialSubdivision2D::cell_id_to_cell_type(home_cell_id, self.num_cells);
            let object = Object {
                id: i as u32,
                control_bits: (home_cell_type << 4 | phantom_cell_bits)
            };

            for (count, cell_id) in phantom_cell_ids.iter().enumerate() {
                cells[offset + count] = cell_id.clone();
                objects[offset + count] = object.clone();
            }
        }
    }

    fn sort_arrays(cell_id_array: &mut Vec<u32>, object_id_array: &mut Vec<Object>, num_cells: u32) {
        // FIXME: This is a naive implementation. We should use a more efficient sorting algorithm
        let mut zipped: Vec<(_, _)> = cell_id_array.iter().cloned().zip(object_id_array.iter().cloned()).collect();
        zipped.sort_by(|a, b| {
            // If cell id is equal, set home cell type first
            if a.0 == b.0 {
                if a.0 == u32::MAX {
                    return std::cmp::Ordering::Equal;
                }
                let current_cell_id = a.0;
                let current_cell_type = SpatialSubdivision2D::cell_id_to_cell_type(current_cell_id, num_cells);
                let cell_type_a = a.1.control_bits >> 4;
                let cell_type_b = b.1.control_bits >> 4;

                if cell_type_a == cell_type_b {
                    // Both are home cells, sort by id
                    return a.1.id.cmp(&b.1.id);
                } else if cell_type_a == current_cell_type {
                    // a is home cell, disregard b as if b also is home cell we preserve order
                    return std::cmp::Ordering::Less;
                } else if cell_type_b == current_cell_type{
                    // b is home cell
                    return std::cmp::Ordering::Greater;
                } else {
                    // Both are phantom cells, sort by id
                    return a.1.id.cmp(&b.1.id);
                }
            }
            return a.0.cmp(&b.0);
        });
        let (sorted_cell_id_array, sorted_object_id_array): (Vec<_>, Vec<_>) = zipped.into_iter().unzip();
        *cell_id_array = sorted_cell_id_array;
        *object_id_array = sorted_object_id_array;
    }

    fn create_collision_cell_list(&self, cell_id_array: &Vec<u32>)
            -> (Vec<CollisionCell>, Vec<CollisionCell>, Vec<CollisionCell>, Vec<CollisionCell>){

        // Count the number of objects in each cell
        let mut objects_per_cell: Vec<u32> = vec![0; self.num_cells.pow(2) as usize];
        let mut current_cell_id = cell_id_array[0];
        let mut count = 0;
        for id in cell_id_array {
            // Early stoping as all empy cells are at the end of the array
            if id == &u32::MAX {
                objects_per_cell[current_cell_id as usize] = count;
                break;
            }
            if id == &current_cell_id {
                count += 1;
            } else {
                objects_per_cell[current_cell_id as usize] = count;
                count = 1;
                current_cell_id = id.clone();
            }
        }

        let mut pass0: Vec<CollisionCell> = Vec::new();
        let mut pass1: Vec<CollisionCell> = Vec::new();
        let mut pass2: Vec<CollisionCell> = Vec::new();
        let mut pass3: Vec<CollisionCell> = Vec::new();

        let mut offset = 0;
        let mut last_processed_cell_id = u32::MAX;
        for cell_id in cell_id_array {
            // Early stoping as all empy cells are at the end of the array
            if cell_id == &u32::MAX {
                break;
            }
            if cell_id == &last_processed_cell_id {
                offset += 1;
                continue;
            }
            last_processed_cell_id = (*cell_id).clone();

            let num_objects_in_cell =  objects_per_cell[*cell_id as usize];
            // No objects in this cell
            if num_objects_in_cell == 0 {
                continue;
            }

            // No collision will happen in this cell as there only is one item
            if num_objects_in_cell == 1 {
                offset += 1;
                continue;
            }

            // Collision will happen in this cell
            let collision = CollisionCell {
                offset: offset.clone() as usize,
                num_objects: num_objects_in_cell,
            };
           
            let cell_type = SpatialSubdivision2D::cell_id_to_cell_type(*cell_id, self.num_cells);
            match cell_type {
                0 => pass0.push(collision),
                1 => pass1.push(collision),
                2 => pass2.push(collision),
                3 => pass3.push(collision),
                _ => panic!("Unexpected cell type"),
            }
            offset += 1;
        }

        return (pass0, pass1, pass2, pass3);
    }

    /// Check if two objects need a detailed collision check.
    /// 
    /// If one of the object home cells, T' is:
    /// - less than pass number
    /// - (and) among the cell types that are common to both objects
    /// we can skip the collision check.
    /// 
    /// Returns true if the detailed collision check can be skipped.
    fn skip_detailed_collision_check(a: &Object, b: &Object, pass_num: u8) -> bool {
        let common_cell_types = a.control_bits & b.control_bits & 0b00001111;

        let home_cell_type_a = a.control_bits >> 4;
        let home_cell_type_bit_flag_a = 1 << home_cell_type_a;
        let object_a_less_than_pass_num = home_cell_type_a < pass_num;
        let object_a_home_cell_type_among_common_cell_types = home_cell_type_bit_flag_a & common_cell_types > 0;

        if object_a_less_than_pass_num && object_a_home_cell_type_among_common_cell_types {
            return true;
        }
        
        let home_cell_type_b = b.control_bits >> 4;
        let home_cell_type_bit_flag_b = 1 << home_cell_type_b;
        let object_b_less_than_pass_num = home_cell_type_b < pass_num;
        let object_b_home_cell_type_among_common_cell_types = home_cell_type_bit_flag_b & common_cell_types > 0;

        if object_b_less_than_pass_num && object_b_home_cell_type_among_common_cell_types {
            return true;
        }

        return false;
    }

    fn run_pass(
            pass_num: u8,
            pass: &Vec<CollisionCell>, object_id_array: &Vec<Object>,
            positions: &mut [Vector3<f32>; 10000], velocities: &mut [Vector3<f32>; 10000],
            mass: &[f32; 10000],
            collision_check_fn: fn(&Vector3<f32>, &Vector3<f32>, f32, &Vector3<f32>, &Vector3<f32>, f32) -> Option<f32>,
            collision_response_fn: fn(f32, &mut Vector3<f32>, &mut Vector3<f32>, &Vector3<f32>, &Vector3<f32>, f32, f32, f32),
            radius: f32) {
        for collision in pass {
            let offset = collision.offset;
            let num_objects = collision.num_objects;
            for i in 0..num_objects {
                
                let pos_a = positions[object_id_array[offset + i as usize].id as usize];
                let vel_a = velocities[object_id_array[offset + i as usize].id as usize];
                let new_pos_a = pos_a + vel_a;
                let a = object_id_array[offset + i as usize];
                
                for j in i+1..num_objects {
                
                    let pos_b = positions[object_id_array[offset + j as usize].id as usize];
                    let vel_b = velocities[object_id_array[offset + j as usize].id as usize];
                    let new_pos_b = pos_b + vel_b;
                    let b = object_id_array[offset + j as usize];
                
                    if SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num) {
                        continue;
                    }
                    match collision_check_fn(&pos_a, &new_pos_a, radius, &pos_b, &new_pos_b, radius) {
                        Some(ttc) if -1.0 <= ttc && ttc <= 1.0  => {
                            let mut va = velocities[a.id as usize].clone();
                            let mut vb = velocities[b.id as usize].clone();
                            let mut pa = positions[a.id as usize].clone();
                            let mut pb = positions[b.id as usize].clone();
                            let massa = mass[a.id as usize];
                            let massb = mass[b.id as usize];
                            collision_response_fn(ttc, &mut va, &mut vb, &mut pa, &mut pb, massa, massb, 1.0);
                            
                            velocities[a.id as usize] = va;
                            velocities[b.id as usize] = vb;
                        },
                        _ => ()
                    }
                }
            }
        }
    }

    /// Given a list of bounding boxes, construct the cell id array for the cell grid
    /// Args:
    /// - bboxes: A list of bounding boxes in the format of (x, y, z, w)
    ///         where x, y is the top left corner and z, w is the width and height.
    ///         Note that w is not the width, z is the width. Additionally, the points
    ///         of the bounding box are expected to be between -1.0 and 1.0. Furthermore,
    ///         each bounding box is expected to be axis-aligned.
    ///          
    pub fn run(&self,
            positions: &mut [Vector3<f32>; 10000],
            velocities: &mut [Vector3<f32>; 10000],
            collision_check_fn: 
                fn( &Vector3<f32>, &Vector3<f32>, f32, 
                    &Vector3<f32>, &Vector3<f32>, f32) -> Option<f32>,
            collision_response_fn: 
                fn(ttc: f32, 
                    &mut Vector3<f32>, &mut Vector3<f32>,
                    &Vector3<f32>, &Vector3<f32>,
                    f32, f32, f32),
            radius: f32,
            mass: &[f32; 10000],
            bounding_spheres: &Vec<Vector3<f32>>,
        ){
        
        let num_of_bounding_sphere = bounding_spheres.len();
        let invalid_cell_id = u32::MAX;
        let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   num_of_bounding_sphere * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
        let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); num_of_bounding_sphere * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];

        self.construct_arrays(&bounding_spheres, &mut cell_id_array, &mut object_id_array);

        // Sort the arrays according to the cell id
        SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, self.num_cells);
        let (pass0, pass1, pass2, pass3) = self.create_collision_cell_list(&cell_id_array);

        if pass0.len() != 0 {
            SpatialSubdivision2D::run_pass(0,&pass0, &object_id_array, positions, velocities, mass, collision_check_fn, collision_response_fn, radius);
        }

        if pass1.len() != 0 {
            SpatialSubdivision2D::run_pass(1,&pass1, &object_id_array, positions, velocities, mass, collision_check_fn, collision_response_fn, radius);
        }

        if pass2.len() != 0 {
            SpatialSubdivision2D::run_pass(2,&pass2, &object_id_array, positions, velocities, mass, collision_check_fn, collision_response_fn, radius);
        }
        if pass3.len() != 0 {
            SpatialSubdivision2D::run_pass(3, &pass3, &object_id_array, positions, velocities, mass, collision_check_fn, collision_response_fn, radius);
        }
    }

}

#[cfg(test)]
mod tests {
    mod test_point_to_cell_id {
        use super::super::SpatialSubdivision2D;
        #[test]
        fn test_point_to_cell_id_top_left() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.0, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 0);
        }
        #[test]
        fn test_point_to_cell_id_bottom_left() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.0, 0.0);
            assert_eq!(ss.point_to_cell_id(point), 380);
        }
        #[test]
        fn test_point_to_cell_id_bottom_right() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(2.0, 0.0);
            assert_eq!(ss.point_to_cell_id(point), 399);
        }
        #[test]
        fn test_point_to_cell_id_top_right() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(2.0, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 19);
        }
        #[test]
        fn test_point_to_cell_id_right_of_border() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.1, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 1);
        }
        #[test]
        fn test_point_to_cell_id_left_of_border() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.09999, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 0);
        }

        #[test]
        fn test_point_to_cell_id_top_of_border() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.0, 1.90001);
            assert_eq!(ss.point_to_cell_id(point), 0);
        }

        #[test]
        fn test_point_to_cell_id_bottom_of_border() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.0, 1.8999);
            assert_eq!(ss.point_to_cell_id(point), 20);
        }
    
        #[test]
        fn test_point_to_cell_id_border_to_bottom() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.0, 1.9);
            assert_eq!(ss.point_to_cell_id(point), 20);
        }
    
        #[test]
        fn test_point_to_cell_id_border_to_diagonal() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.1, 1.9);
            assert_eq!(ss.point_to_cell_id(point), 21);
        }

        #[test]
        fn test_point_to_cell_id_uneven_cell_size_division() {
            let cell_size = 0.15;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(2.0, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 13);
        }

        #[test]
        fn test_point_to_cell_id_live_scenario_3() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.1029385, 0.082769275);
            assert_eq!(ss.point_to_cell_id(point), 381);
        }
    }

    mod test_compute_phantom_cell_ids {
        use std::vec;
        use cgmath::num_traits::Float;
        use super::super::SpatialSubdivision2D;

        #[test]
        fn test_compute_phantom_cell_ids_span_no_other_than_home() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(0.05, 1.95, radius);
            let expected_result = vec![0];
            let ss = SpatialSubdivision2D::new(1.0);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_span_neighbor_row_cell() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(0.08, 0.05, radius);
            let expected_result = vec![380, 381];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_2_by_2_top_left() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(0.1, 1.9, radius);
            let expected_result = vec![21,0,1,20];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_2_by_2_bottom_right() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(1.9, 0.1, radius);
            let expected_result = vec![399,378,379,398];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_2_by_2_bottom_left() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(0.1, 0.1, radius);
            let expected_result = vec![381,360,361,380];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_2_by_2_top_right() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(1.9, 1.9, radius);
            let expected_result = vec![39,18,19,38];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_middle() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let num_cells = (2.0 / cell_size).ceil() as u32;
            let bbox = cgmath::Vector3::new(1.0, 1.0, radius);
            let expected_result = vec![210,189,190,209];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox, cell_size, num_cells);
            assert_eq!(result, expected_result);
        }
    }

    mod test_get_cell_id_center {
        use cgmath::InnerSpace;

        use super::super::SpatialSubdivision2D;
        #[test]
        fn test_get_cell_id_center_top_left() {
            let cell_size = 0.1;
            let num_cells = 20;
            let cell_id = 0;
            let allowed_diff = 0.0001;
            let expected_result = cgmath::Vector2::new(0.05, 1.95);
            let result = SpatialSubdivision2D::get_cell_id_center(cell_id, cell_size, num_cells);
            let diff = (result - expected_result).magnitude();
            assert!(diff < allowed_diff);
        }

        #[test]
        fn test_get_cell_id_center_bottom_right() {
            let cell_size = 0.1;
            let num_cells = 20;
            let cell_id = 399;
            let allowed_diff = 0.0001;
            let expected_result = cgmath::Vector2::new(1.95, 0.05);
            let result = SpatialSubdivision2D::get_cell_id_center(cell_id, cell_size, num_cells);
            let diff = (result - expected_result).magnitude();
            assert!(diff < allowed_diff);
        }
    }

    mod test_cell_id_to_cell_type {
        use super::super::SpatialSubdivision2D;

        #[test]
        fn test_cell_id_to_cell_type_top_left() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let mut cell_id = 0;
            let mut expected_result = 0;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 1;
            expected_result = 1;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 20;
            expected_result = 2;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 21;
            expected_result = 3;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);
        }

        #[test]
        fn test_cell_id_to_cell_type_top_right() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let mut cell_id = 18;
            let mut expected_result = 0;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 19;
            expected_result = 1;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 38;
            expected_result = 2;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 39;
            expected_result = 3;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);
        }

        #[test]
        fn test_cell_id_to_cell_type_real_scenario_1() {
            let cell_size = 0.44547725;
            // let num_cells = 5;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let mut cell_id = 0;
            let mut expected_result = 0;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 1;
            expected_result = 1;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 2;
            expected_result = 0;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 3;
            expected_result = 1;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 4;
            expected_result = 0;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            let mut cell_id = 5;
            let mut expected_result = 2;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);

            cell_id = 6;
            expected_result = 3;
            assert_eq!(SpatialSubdivision2D::cell_id_to_cell_type(cell_id, num_cells), expected_result);
        }
    }

    mod test_construct_arrays {
        use crate::simulation::spatial_subdivision::{Object, SpatialSubdivision2D, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};
        
        #[test]
        fn test_construct_arrays_one_object_max_num_phantom_cells(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id = 0;
            let phantom_cell_id_1 = 1;
            let phantom_cell_id_2 = 20;
            let phantom_cell_id_3 = 21;
            let bboxes = vec![
                cgmath::Vector3::new(-0.901, 0.901, radius),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0);
            assert_eq!(object_id_array[1].id, 0);
            assert_eq!(object_id_array[2].id, 0);
            assert_eq!(object_id_array[3].id, 0);
            for i in 4..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }
            
            assert_eq!(object_id_array[0].control_bits, 0b001111);
            assert_eq!(object_id_array[1].control_bits, 0b001111);
            assert_eq!(object_id_array[2].control_bits, 0b001111);
            assert_eq!(object_id_array[3].control_bits, 0b001111);
            for i in 4..object_id_array.len() {
                assert_eq!(object_id_array[i].control_bits, 0);
            }

            assert_eq!(cell_id_array[0], home_cell_id);
            assert_eq!(cell_id_array[1], phantom_cell_id_1);
            assert_eq!(cell_id_array[2], phantom_cell_id_2);
            assert_eq!(cell_id_array[3], phantom_cell_id_3);
            for i in 4..cell_id_array.len() {
                assert_eq!(cell_id_array[i], invalid_cell_id);
            }

        }

        #[test]
        fn test_construct_arrays_one_object_1_phantom_cell() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id = 379;
            let phantom_cell_id_1 = 399;
            let bboxes = vec![
                cgmath::Vector3::new(0.95, -0.89, radius),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0);
            assert_eq!(object_id_array[1].id, 0);
            for i in 2..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }

            assert_eq!(object_id_array[0].control_bits, 0b011010);
            assert_eq!(object_id_array[1].control_bits, 0b011010);
            assert_eq!(object_id_array[2].control_bits, 0);
            assert_eq!(object_id_array[3].control_bits, 0);
            for i in 4..object_id_array.len() {
                assert_eq!(object_id_array[i].control_bits, 0);
            }

            assert_eq!(cell_id_array[0], home_cell_id);
            assert_eq!(cell_id_array[1], phantom_cell_id_1);
            for i in 2..cell_id_array.len() {
                assert_eq!(cell_id_array[i], invalid_cell_id);
            }
        }

        #[test]
        fn test_construct_arrays_two_overlapping_bboxes() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id_1 = 20;
            let home_cell_id_2 = 21;
            let phantom_cell_id_1 = 21;
            let phantom_cell_id_2 = 22;
            let bboxes = vec![
                cgmath::Vector3::new(-0.901, 0.85, radius),
                cgmath::Vector3::new(-0.801, 0.85, radius),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0); // Home cell 1
            assert_eq!(object_id_array[1].id, 0); // Phantom cell 1
            assert_eq!(object_id_array[2].id, invalid_cell_id);
            assert_eq!(object_id_array[3].id, invalid_cell_id);
            assert_eq!(object_id_array[4].id, 1); // Home cell 2
            assert_eq!(object_id_array[5].id, 1); // Phantom cell 2
            for i in 6..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }

            assert_eq!(object_id_array[0].control_bits, 0b101100);
            assert_eq!(object_id_array[1].control_bits, 0b101100);
            assert_eq!(object_id_array[2].control_bits, 0);
            assert_eq!(object_id_array[3].control_bits, 0);
            assert_eq!(object_id_array[4].control_bits, 0b111100);
            assert_eq!(object_id_array[5].control_bits, 0b111100);
            assert_eq!(object_id_array[6].control_bits, 0);
            assert_eq!(object_id_array[7].control_bits, 0);
            for i in 8..object_id_array.len() {
                assert_eq!(object_id_array[i].control_bits, 0);
            }

            assert_eq!(cell_id_array[0], home_cell_id_1);
            assert_eq!(cell_id_array[1], phantom_cell_id_1);
            assert_eq!(cell_id_array[2], invalid_cell_id);
            assert_eq!(cell_id_array[3], invalid_cell_id);
            assert_eq!(cell_id_array[4], home_cell_id_2);
            assert_eq!(cell_id_array[5], phantom_cell_id_2);
            for i in 6..cell_id_array.len() {
                assert_eq!(cell_id_array[i], invalid_cell_id);
            }
        }

        #[test]
        fn test_construct_arrays_live_scenario_1() {
            // radius = 0.1 and velocity = 0.01
            let cell_size = 0.44547725;
            // let num_cells = 5;
            let ss = SpatialSubdivision2D::new(cell_size);
            let bboxes = vec![
                cgmath::Vector3::new(-0.24500024, 0.0, 0.14849241),
                cgmath::Vector3::new(0.24500024, 0.0, 0.14849241)];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            let expected_cell_id_array = vec![11,6,12, u32::MAX, 12,7,13, u32::MAX];
            assert_eq!(expected_cell_id_array, cell_id_array);

            let expected_object_invalid = Object{id: u32::MAX, control_bits: 0};
            let expected_object_0 = Object{id: 0, control_bits: 0b011011};
            let expected_object_1 = Object{id: 1, control_bits: 0b000111};

            assert_eq!(expected_object_0, object_id_array[0]);
            assert_eq!(expected_object_0, object_id_array[1]);
            assert_eq!(expected_object_0, object_id_array[2]);
            assert_eq!(expected_object_invalid, object_id_array[3]);
            assert_eq!(expected_object_1, object_id_array[4]);
            assert_eq!(expected_object_1, object_id_array[5]);
            assert_eq!(expected_object_1, object_id_array[6]);
            assert_eq!(expected_object_invalid, object_id_array[7]);            
        }

        #[test]
        fn test_construct_arrays_live_scenario_2() {
            // radius = 0.1 and velocity = 0.01
            let cell_size = 0.44547725;
            // let num_cells = 5;
            let ss = SpatialSubdivision2D::new(cell_size);
            let bboxes = vec![
                cgmath::Vector3::new(-0.18500021, 0.0, 0.14849241),
                cgmath::Vector3::new(0.18500021, 0.0, 0.14849241)];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            let expected_cell_id_array = vec![11,6,7,12,12,7,u32::MAX, u32::MAX];
            assert_eq!(expected_cell_id_array, cell_id_array);

            let object_invalid = Object{id: u32::MAX, control_bits: 0};
            let object_0 = Object{id: 0, control_bits: 31};
            let object_1 = Object{id: 1, control_bits: 5};
            let expected_object_id_array = vec![
                object_0, object_0, object_0, object_0, 
                object_1, object_1, object_invalid, object_invalid];
         
            assert_eq!(expected_object_id_array, object_id_array);
        }
    }

    mod test_sort_arrays {
        use std::u32;

        use crate::simulation::spatial_subdivision::{Object, SpatialSubdivision2D, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        #[test]
        fn test_sort_arrays() {
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let bboxes = vec![
                cgmath::Vector3::new(-0.901, 0.901, radius),
                cgmath::Vector3::new(-0.901, 0.95, radius),
                cgmath::Vector3::new(-0.95, 0.801, radius),
            ];

            let expected_cell_id_array =  [0,0,1,1,20,20,21,40,u32::MAX,u32::MAX,u32::MAX,u32::MAX];
            let expeced_object_id_array = [0,1,0,1, 2, 0, 0, 2,u32::MAX,u32::MAX,u32::MAX,u32::MAX];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);

            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let object_ids = object_id_array.iter().map(|object| object.id).collect::<Vec<u32>>();

            assert_eq!(cell_id_array, expected_cell_id_array); 
            assert_eq!(object_ids, expeced_object_id_array);
        }

        #[test]
        fn test_sort_arrays_live_scenario_1() {
            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let mut cell_id_array = vec![11,6,12, u32::MAX, 12,7,13, u32::MAX];
            let mut object_id_array = vec! [
                Object { id: 0, control_bits: 0b011011 }, Object { id: 0, control_bits: 0b011011 },
                Object { id: 0, control_bits: 0b011011 }, Object { id: u32::MAX, control_bits: 0 },
                Object { id: 1, control_bits: 0b000111 }, Object { id: 1, control_bits: 0b000111 },
                Object { id: 1, control_bits: 0b000111 }, Object { id: u32::MAX, control_bits: 0 }];

            let expected_cell_id_array = vec![6, 7, 11, 12, 12, 13, u32::MAX, u32::MAX];
            let expected_object_id_array = vec![
                Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
                Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
                Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
                Object { id: u32::MAX, control_bits: 0 }, Object { id: u32::MAX, control_bits: 0 }];

            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            assert_eq!(cell_id_array, expected_cell_id_array);
            assert_eq!(object_id_array, expected_object_id_array);
        }

        #[test]
        fn test_sort_arrays_live_scenario_2() {
            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let mut cell_id_array = vec![11, 6, 7, 12, 12, 7, u32::MAX, u32::MAX];
            let object_a = Object { id: 0, control_bits: 31 };
            let object_b = Object { id: 1, control_bits: 5 };
            let object_none = Object { id: u32::MAX, control_bits: 0 };
            let mut object_id_array = vec![
                object_a.clone(), object_a.clone(), object_a.clone(), object_a.clone(),
                object_b.clone(), object_b.clone(), object_none.clone(), object_none.clone()];

            let expected_cell_id_array = vec![6, 7, 7, 11, 12, 12, u32::MAX, u32::MAX];
            let expected_object_id_array = vec![
                object_a.clone(), object_a.clone(), object_b.clone(), object_a.clone(),
                object_b.clone(), object_a.clone(), object_none.clone(), object_none.clone()];

            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            assert_eq!(cell_id_array, expected_cell_id_array);
            assert_eq!(object_id_array, expected_object_id_array);
        }
    }

    mod test_get_neighbor_cell_ids {
        use crate::simulation::spatial_subdivision::SpatialSubdivision2D;

        #[test]
        fn test_get_neighbor_cell_ids_top_left() {
            let center_cell = 0;
            let num_cells = 20;
            let expected_result = [u32::MAX, u32::MAX, u32::MAX, u32::MAX, 1, u32::MAX, 20, 21];
            let neighbors = SpatialSubdivision2D::get_neighbor_cell_ids(center_cell, num_cells);
            assert_eq!(neighbors, expected_result);
        }

        #[test]
        fn test_get_neighbor_cell_ids_top_right() {
            let center_cell = 19;
            let num_cells = 20;
            let expected_result = [u32::MAX, u32::MAX, u32::MAX, 18, u32::MAX, 38, 39, u32::MAX];
            let neighbors = SpatialSubdivision2D::get_neighbor_cell_ids(center_cell, num_cells);
            assert_eq!(neighbors, expected_result);
        }

        #[test]
        fn test_get_neighbor_cell_ids_bottom_left() {
            let center_cell = 380;
            let num_cells = 20;
            let expected_result = [u32::MAX, 360, 361, u32::MAX, 381, u32::MAX, u32::MAX, u32::MAX];
            let neighbors = SpatialSubdivision2D::get_neighbor_cell_ids(center_cell, num_cells);
            assert_eq!(neighbors, expected_result);
        }

        #[test]
        fn test_get_neighbor_cell_ids_bottom_right() {
            let center_cell = 399;
            let num_cells = 20;
            let expected_result = [378, 379, u32::MAX, 398, u32::MAX, u32::MAX, u32::MAX, u32::MAX];
            let neighbors = SpatialSubdivision2D::get_neighbor_cell_ids(center_cell, num_cells);
            assert_eq!(neighbors, expected_result);
        }

        #[test]
        fn test_get_neighbor_cell_ids_not_corner() {
            let center_cell = 21;
            let num_cells = 20;
            let expected_result = [0, 1, 2, 20, 22, 40, 41, 42];
            let neighbors = SpatialSubdivision2D::get_neighbor_cell_ids(center_cell, num_cells);
            assert_eq!(neighbors, expected_result);
        }
    }

    mod test_create_collision_cell_list {
        use crate::simulation::spatial_subdivision::{CollisionCell, Object, SpatialSubdivision2D, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        #[test]
        fn test_create_collision_cell_list_no_collisions(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let bboxes = vec![
                cgmath::Vector3::new(-0.901, 0.95, radius),
                cgmath::Vector3::new(-0.901, 0.85, radius),
                cgmath::Vector3::new(-0.75, 0.901, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_one_collision_pass_0(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.95, 0.95, radius),
                cgmath::Vector3::new(-0.95, 0.95, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert_eq!(pass0.len(), 1);
            assert_eq!(pass0[0].offset, expected_result.offset);
            assert_eq!(pass0[0].num_objects, expected_result.num_objects);
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_one_collision_pass_1(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.85, 0.95,radius),
                cgmath::Vector3::new(-0.85, 0.95,radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert_eq!(pass1.len(), 1);
            assert_eq!(pass1[0].offset, expected_result.offset);
            assert_eq!(pass1[0].num_objects, expected_result.num_objects);
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_one_collision_pass_2(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.95, 0.85, radius),
                cgmath::Vector3::new(-0.95, 0.85, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert_eq!(pass2.len(), 1);
            assert_eq!(pass2[0].offset, expected_result.offset);
            assert_eq!(pass2[0].num_objects, expected_result.num_objects);
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_one_collision_pass_3(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.85, 0.85, radius),
                cgmath::Vector3::new(-0.85, 0.85, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert_eq!(pass3.len(), 1);
            assert_eq!(pass3[0].offset, expected_result.offset);
            assert_eq!(pass3[0].num_objects, expected_result.num_objects);
        }

        #[test]
        fn test_create_collision_cell_list_one_collisions_different_home_cells(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.95, 0.95, radius),
                cgmath::Vector3::new(-0.89, 0.89, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert_eq!(pass0.len(), 1);
            assert_eq!(pass0[0].offset, expected_result.offset);
            assert_eq!(pass0[0].num_objects, expected_result.num_objects);
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_two_collisions_different_home_cells(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result_pass_0 = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let expected_result_pass_1 = CollisionCell {
                offset: 2,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.95, 0.95, radius),
                cgmath::Vector3::new(-0.89, 0.89, radius),
                cgmath::Vector3::new(-0.85, 0.95, radius),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert_eq!(pass0.len(), 1);
            assert_eq!(pass0[0].offset, expected_result_pass_0.offset);
            assert_eq!(pass0[0].num_objects, expected_result_pass_0.num_objects);
            assert_eq!(pass1.len(), 1);
            assert_eq!(pass1[0].offset, expected_result_pass_1.offset);
            assert_eq!(pass1[0].num_objects, expected_result_pass_1.num_objects);
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_pass_2_different_sectors(){
            let cell_size = 0.1;
            let radius = cell_size / (2.0 * 1.5);
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let expected_result_1 = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let expected_result_2 = CollisionCell {
                offset: 2,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector3::new(-0.95, 0.85, radius),
                cgmath::Vector3::new(-0.95, 0.85, radius),
                cgmath::Vector3::new(-0.95, 0.65, radius),
                cgmath::Vector3::new(-0.95, 0.65, radius),

            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.construct_arrays(&bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert_eq!(pass2.len(), 2);
            assert_eq!(pass2[0].offset, expected_result_1.offset);
            assert_eq!(pass2[0].num_objects, expected_result_1.num_objects);
            assert_eq!(pass2[1].offset, expected_result_2.offset);
            assert_eq!(pass2[1].num_objects, expected_result_2.num_objects);
            assert!(pass3.is_empty());
        }


        #[test]
        fn test_create_collision_cell_list_different_home_cells_2() {
            let cell_id_array = vec![6, 6, 7, 7, u32::MAX, u32::MAX, u32::MAX, u32::MAX];
            // Not needed but explains the state of the objects in the cells
            // let object_id_array =  vec![
            //     Object { id: 0, control_bits: 0b111100 }, Object { id: 1, control_bits: 0b101100 },
            //     Object { id: 1, control_bits: 0b101100 }, Object { id: 0, control_bits: 0b111100 },
            //     Object { id: u32::MAX, control_bits: 0 }, Object { id: u32::MAX, control_bits: 0 },
            //     Object { id: u32::MAX, control_bits: 0 }, Object { id: u32::MAX, control_bits: 0 }];

            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let (pass0, pass1, pass2, pass3) = ss.create_collision_cell_list(&cell_id_array);

            let expected_pass_2 = vec![CollisionCell { offset: 2, num_objects: 2}];
            let expected_pass_3 = vec![CollisionCell { offset: 0, num_objects: 2}];
            
            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert_eq!(pass2, expected_pass_2);
            assert_eq!(pass3, expected_pass_3);
        }

        #[test]
        fn test_create_collision_cell_list_live_scenario_1() {
            let cell_id_array = vec![6, 7, 11, 12, 12, 13, u32::MAX, u32::MAX]; // sorted
            // Not needed but explains the state of the objects in the cells
            // let object_id_array =  vec![
            //     Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
            //     Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
            //     Object { id: 0, control_bits: 0b011011 }, Object { id: 1, control_bits: 0b000111 },
            //     Object { id: u32::MAX, control_bits: 0 }, Object { id: u32::MAX, control_bits: 0 }];

            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let (pass0, pass1, pass2, pass3) = ss.create_collision_cell_list(&cell_id_array);

            let expected_pass_0 = vec![CollisionCell { offset: 3, num_objects: 2}];

            assert_eq!(pass0, expected_pass_0);
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert!(pass3.is_empty());
        }

        #[test]
        fn test_create_collision_cell_list_live_scenario_2() {
            // num cells: 5
            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let cell_id_array =  vec!(6, 7, 7, 11, 12, 12, u32::MAX, u32::MAX);

            let expected_pass_0 = vec![CollisionCell { offset: 4, num_objects: 2}];
            let expected_pass_2 = vec![CollisionCell { offset: 1, num_objects: 2}];
            let (pass0, pass1, pass2, pass3) = ss.create_collision_cell_list(&cell_id_array);

            assert_eq!(pass0, expected_pass_0);
            assert!(pass1.is_empty());
            assert_eq!(pass2, expected_pass_2);
            assert!(pass3.is_empty());
        }
    }

    mod test_full {
        

        use cgmath::Vector3;

        use crate::simulation::spatial_subdivision::{CollisionCell, Object, SpatialSubdivision2D, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        #[test]
        fn test_full_live_scenario_3() {
            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let num_cells = ss.num_cells; // Private

            let bounding_spheres = vec!(Vector3::new(0.42664176, 0.40857708, 0.14619982), Vector3::new(0.50925887, 0.55335414, 0.14969368));
            let num_of_bounding_sphere = bounding_spheres.len();
            let _positions =    vec!(Vector3::new(0.4240127, 0.40645462, 0.0), Vector3::new(0.5149141, 0.55484885, 0.0));
            let _new_positions =    vec!(Vector3::new(0.42927086, 0.41069952, 0.0), Vector3::new(0.5036037, 0.5518595, 0.0));
            
            let _velocities =       vec!(Vector3::new(0.005258184, 0.0042448994, 0.0), Vector3::new(-0.011310411, -0.0029893694, 0.0));
            let _new_velocities =       vec!(Vector3::new(0.005258184, 0.0042448994, 0.0), Vector3::new(-0.011310411, -0.0029893694, 0.0));

            let expected_phantom_cells_0 = vec!(8,7);
            let expected_phantom_cells_1 = vec!(8,3);
            let offset = Vector3::new(1.0, 1.0, 0.0);
            let phantom_cells_0 = ss.compute_phantom_cell_ids(bounding_spheres[0] + offset, cell_size, num_cells);
            let phantom_cells_1 = ss.compute_phantom_cell_ids(bounding_spheres[1] + offset, cell_size, num_cells);

            assert_eq!(phantom_cells_0, expected_phantom_cells_0);
            assert_eq!(phantom_cells_1, expected_phantom_cells_1);

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   num_of_bounding_sphere * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); num_of_bounding_sphere * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            ss.construct_arrays(&bounding_spheres, &mut cell_id_array, &mut object_id_array);

            let object_0 = Object { id: 0, control_bits: 0b111100 };
            let object_1 = Object { id: 1, control_bits: 0b111010 };
            let object_invalid = Object { id: u32::MAX, control_bits: 0 };
            let expected_cell_id_array = vec!(8, 7, u32::MAX, u32::MAX, 8, 3, u32::MAX, u32::MAX);
            let expected_object_id_array = vec!(
                object_0.clone(), object_0.clone(), object_invalid.clone(), object_invalid.clone(),
                object_1.clone(), object_1.clone(), object_invalid.clone(), object_invalid.clone());

            assert_eq!(cell_id_array, expected_cell_id_array);
            assert_eq!(object_id_array, expected_object_id_array);
            

            let expected_sorted_cell_id_array = vec!(3, 7, 8, 8, u32::MAX, u32::MAX, u32::MAX, u32::MAX);
            let expected_sorted_object_id_array = vec!(
                object_1.clone(), object_0.clone(), object_0.clone(), object_1.clone(),
                object_invalid.clone(), object_invalid.clone(), object_invalid.clone(), object_invalid.clone());
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, num_cells);

            assert_eq!(cell_id_array, expected_sorted_cell_id_array);
            assert_eq!(object_id_array, expected_sorted_object_id_array);
            
            let expected_pass_3 = vec!(CollisionCell { offset: 2, num_objects: 2 });
            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert!(pass2.is_empty());
            assert_eq!(pass3, expected_pass_3);
        }

        #[test]
        fn test_full_live_scenario_4(){
            let cell_size = 0.44547725;
            let ss = SpatialSubdivision2D::new(cell_size);
            let bounding_spheres = vec![
                cgmath::Vector3::new(-0.02122597, 0.08115901, 0.14885814),
                cgmath::Vector3::new(-0.055799574, 0.26585513, 0.14463444),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id; bounding_spheres.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bounding_spheres.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            
            let object_a = Object{id: 0, control_bits: 0b001111};
            let object_b = Object{id: 1, control_bits: 0b101100};
            let expected_object_invalid = Object{id: u32::MAX, control_bits: 0};
            let expected_object_id_array = vec![
                object_a.clone(), object_a.clone(), object_a.clone(), object_a.clone(),
                object_b.clone(), object_b.clone(), expected_object_invalid.clone(), expected_object_invalid.clone()];
            let expected_cell_id_array = vec![12,6,7,11,7,6,u32::MAX,u32::MAX];
            
            ss.construct_arrays(&bounding_spheres, &mut cell_id_array, &mut object_id_array);

            assert_eq!(expected_cell_id_array, cell_id_array);
            assert_eq!(expected_object_id_array, object_id_array);

            let expected_sorted_cell_id_array = vec![6, 6, 7, 7, 11, 12, u32::MAX, u32::MAX];
            let expected_sorted_object_id_array = vec![
                object_a.clone(), object_b.clone(), object_b.clone(), object_a.clone(),
                object_a.clone(), object_a.clone(), expected_object_invalid.clone(), expected_object_invalid.clone()];

            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array, ss.num_cells);

            assert_eq!(expected_sorted_cell_id_array, cell_id_array);
            assert_eq!(expected_sorted_object_id_array, object_id_array);

            let expected_pass_2 = vec![CollisionCell { offset: 2, num_objects: 2 }];
            let expected_pass_3 = vec![CollisionCell { offset: 0, num_objects: 2 }];

            let passes = ss.create_collision_cell_list(&cell_id_array);
            let (pass0, pass1, pass2, pass3) = passes;

            assert!(pass0.is_empty());
            assert!(pass1.is_empty());
            assert_eq!(pass2, expected_pass_2);
            assert_eq!(pass3, expected_pass_3);

            let pass2_object_a = object_id_array[2];
            let pass2_object_b = object_id_array[3];
            let should_skip_pass_2 = SpatialSubdivision2D::skip_detailed_collision_check(&pass2_object_a, &pass2_object_b, 2);
            assert_eq!(should_skip_pass_2, false);

            let pass3_object_a = object_id_array[0];
            let pass3_object_b = object_id_array[1];
            let should_skip_pass_3 = SpatialSubdivision2D::skip_detailed_collision_check(&pass3_object_a, &pass3_object_b, 3);
            assert_eq!(should_skip_pass_3, true);
      }
    }

    mod test_collision_check {
        use crate::simulation::spatial_subdivision::{Object, SpatialSubdivision2D};
        #[test]
        fn test_skip_detailed_collision_check_1() {
            let a = Object  {
                id: 0,
                control_bits: 0b111100,
            };
            let b = Object {
                id: 1,
                control_bits: 0b101100,
            };

            let mut pass_num = 2;
            let mut result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, false);

            pass_num = 3;
            result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, true);
        }

        #[test]
        fn test_skip_detailed_collision_check_2() {
            let a = Object  {
                id: 0,
                control_bits: 0b000101,
            };
            let b = Object {
                id: 1,
                control_bits: 0b100101,
            };

            let mut pass_num = 0;
            let mut result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, false);

            pass_num = 2;
            result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, true);
        }

        #[test]
        fn test_skip_detailed_collision_check_live_scenario_2() {
            let a = Object  {
                id: 0,
                control_bits: 31,
            };
            let b = Object {
                id: 1,
                control_bits: 5,
            };

            let mut pass_num = 0;
            let mut result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, false);

            pass_num = 2;
            result = SpatialSubdivision2D::skip_detailed_collision_check(&a, &b, pass_num);
            assert_eq!(result, true);
        }
    }

}