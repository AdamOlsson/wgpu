use core::num;
use std::cell;

use cgmath::{num_traits::Pow, Vector2, Vector4};
use rand::seq::index;

/* 
Cell ID layout:
-------
|0|1|2|
|3|4|5|
|6|7|8|
-------
*/

// const HOME_CELL_TYPE_1: u8 = 0b00;
// const HOME_CELL_TYPE_2: u8 = 0b01;
// const HOME_CELL_TYPE_3: u8 = 0b10;
// const HOME_CELL_TYPE_4: u8 = 0b11;
// const HOME_CELL_TYPES: [u8; 4] = [HOME_CELL_TYPE_1, HOME_CELL_TYPE_2, HOME_CELL_TYPE_3, HOME_CELL_TYPE_4];

const PHANTOM_CELL_TYPE_1: u8 = 0b0001;
const PHANTOM_CELL_TYPE_2: u8 = 0b0010;
const PHANTOM_CELL_TYPE_3: u8 = 0b0100;
const PHANTOM_CELL_TYPE_4: u8 = 0b1000;

const SPATIAL_SUBDIVISION_2D_CELL_OFFSET: usize = 4;


#[derive(Clone, Copy, Debug)]
struct Object {
    id: u32,
    control_bits: u8,
}

#[derive(Clone, Copy, Debug)]
struct CollisionCell {
    offset: usize,
    num_objects: u32,
    // num_home_ids: u32,
    // num_phantom_ids: u32,
}

struct SpatialSubdivision2D {
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

    /// Compute the cell ids that are covered by the bounding box.
    /// 
    /// Given a bounding box, compute the cell ids that are covered by the bounding box.
    /// Assumes that the given bounding box have points between 0.0 and 2.0 and that
    /// the bounding box is axis-aligned.
    /// 
    /// Because the bounding box is axis-aligned, we can assume that the top left and
    /// top right point lie on the same row. Therefore, any cell id between the top left
    /// and top right cell id is considered to be covered by the bounding box. The same
    /// logic applies to the bottom left and bottom right cell id and the middle cell ids.
    /// 
    /// Returns a list of cell ids that are covered by the bounding box.
    fn compute_phantom_cell_ids(&self, bbox: Vector4<f32>) -> Vec<u32>{
        let top_left = Vector2::new(bbox.x, bbox.y);
        let top_right = Vector2::new(bbox.x + bbox.z, bbox.y);
        let bottom_left = Vector2::new(bbox.x, bbox.y - bbox.w);
        let bottom_right = Vector2::new(bbox.x + bbox.z, bbox.y - bbox.w);

        let top_left_cell_id = self.point_to_cell_id(top_left);
        let top_right_cell_id = self.point_to_cell_id(top_right);
        let top_cell_ids: Vec<u32> = (top_left_cell_id..=top_right_cell_id).collect();

        let bottom_left_cell_id = self.point_to_cell_id(bottom_left);
        let bottom_right_cell_id = self.point_to_cell_id(bottom_right);
        let bottom_cell_ids: Vec<u32> = (bottom_left_cell_id..=bottom_right_cell_id).collect();

        // Because the bboxes are axis-aligned, we can assume that the top and bottom
        // cell ids have an equal number of cells
        if bottom_cell_ids.len() != top_cell_ids.len() {
            panic!("Top and bottom cell ids have different lengths");
        }
        // In the scenario we have a bbox covers multiple rows, we need to compute the middle cell ids
        let mut middle_cell_ids: Vec<Vec<u32>> = Vec::new();
        top_cell_ids
            .iter()
            .zip(bottom_cell_ids.iter())
            .for_each(
                |(top, bot)| {
                    // println!("{:?} {:?}", top, bot);
                    middle_cell_ids.push( (*top..=*bot).step_by(self.num_cells as usize).collect());
            }
        );

        // Merge all cell ids and remove duplicates
        let mut phantom_cell_ids: Vec<u32> = top_cell_ids.iter().chain(bottom_cell_ids.iter()).chain(middle_cell_ids.iter().flatten()).copied().collect();
        phantom_cell_ids.sort();
        phantom_cell_ids.dedup();
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
    fn cell_id_to_cell_type(&self, cell_id: u32) -> u8 {
        let row = cell_id % 2;
        let col = ((cell_id / self.num_cells) % 2)*2;
        return (row + col).try_into().unwrap();
    }

    fn compute_home_cell_id(&self, bbox: Vector4<f32>) -> u32 {
        let center_x = bbox.x + (bbox.z / 2.0);
        let center_y = bbox.y - (bbox.w / 2.0);
        return self.point_to_cell_id(Vector2::new(center_x, center_y));
    }

    fn constuct_arrays(&self, bboxes: Vec<Vector4<f32>>, cells: &mut Vec<u32>, objects: &mut Vec<Object>) {
        for (i, bbox) in bboxes.iter().enumerate() {
            let offset = i * SPATIAL_SUBDIVISION_2D_CELL_OFFSET;
            // Offset the bbox to have 0.0 and bottom left corner
            let bbox_offset = bbox + Vector4::new(1.0, 1.0, 0.0, 0.0);
            
            let home_cell_id = self.compute_home_cell_id(bbox_offset);
            let phantom_cell_ids = self.compute_phantom_cell_ids(bbox_offset);
            
            // Note that the home cell id is included in the phantom cell ids
            debug_assert!(phantom_cell_ids.len() <= 4, "Did not expect bounding box volume to cover more than 4 cells.");

            // Assign first cell id to home cell id
            cells[offset] = home_cell_id;
            objects[offset].id = i as u32;
            let home_cell_type = self.cell_id_to_cell_type(home_cell_id);
            objects[offset].control_bits |= home_cell_type << 4;

            let mut phantom_offset = 1; // Because index 0 is the home cell id
            for cell_id in phantom_cell_ids.iter() {
                if cell_id == &home_cell_id {
                    continue;
                }
                cells[offset + phantom_offset] = cell_id.clone();
                objects[offset + phantom_offset].id = i as u32;

                let phantom_cell_type = self.cell_id_to_cell_type(cell_id.clone());
                objects[offset + phantom_offset].control_bits |= 1 << phantom_cell_type;
                objects[offset + phantom_offset].control_bits |= home_cell_type << 4;

                phantom_offset += 1;
            }
        }
    }

    fn sort_arrays(cell_id_array: &mut Vec<u32>, object_id_array: &mut Vec<Object>) {
        // FIXME: This is a naive implementation. We should use a more efficient sorting algorithm
        let mut zipped: Vec<(_, _)> = cell_id_array.iter().cloned().zip(object_id_array.iter().cloned()).collect();
        zipped.sort_by(|a, b| a.0.cmp(&b.0));
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
        let mut last_processed_id = usize::MAX;
        for id in cell_id_array {
            // Early stoping as all empy cells are at the end of the array
            if id == &u32::MAX {
                break;
            }
            println!("offset {:?}, {:?}", offset, id);
            let index = *id as usize;
            if index == last_processed_id {
                offset += 1;
                continue;
            }
            last_processed_id = index.clone();

            // No objects in this cell
            if objects_per_cell[index] == 0 {
                continue;
            }

            // No collision will happen in this cell as there only is one item
            if objects_per_cell[index] == 1 {
                offset += 1;
                continue;
            }
            println!("Add collision");
            // Collision will happen in this cell
            let num_objects_in_current_cell = objects_per_cell[index];
            let collision = CollisionCell {
                offset: offset.clone() as usize,
                num_objects: num_objects_in_current_cell,
            };
           
            let cell_type = self.cell_id_to_cell_type(index as u32);
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

    /// Given a list of bounding boxes, construct the cell id array for the cell grid
    /// 
    /// Args:
    /// - bboxes: A list of bounding boxes in the format of (x, y, z, w)
    ///         where x, y is the top left corner and z, w is the width and height.
    ///         Note that w is not the width, z is the width. Additionally, the points
    ///         of the bounding box are expected to be between -1.0 and 1.0. Furthermore,
    ///         each bounding box is expected to be axis-aligned.
    ///          
    fn create_spatial_subdivision(&mut self, bboxes: Vec<Vector4<f32>>) {
        // TODO: Make sure the grid cell size is as large as the largest velocity
        let invalid_cell_id = u32::MAX;
        let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
        let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];

        self.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
        
        // Sort the arrays according to the cell id
        SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);
                    
    }

    fn construct_passes(&mut self, bboxes: Vec<Vector4<f32>>) {
        self.create_spatial_subdivision(bboxes);
    }
}

#[cfg(test)]
mod tests {
    use super::SpatialSubdivision2D;

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
        fn test_point_to_cell_id_border_to_right() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);
            let point = cgmath::Vector2::new(0.1, 2.0);
            assert_eq!(ss.point_to_cell_id(point), 1);
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
    }

    mod test_compute_phantom_cell_ids {
        use std::vec;

        use super::super::SpatialSubdivision2D;

        #[test]
        fn test_compute_phantom_cell_ids_span_single_cell() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(0.1, 2.0, 0.05, 0.05);
            let expected_result = vec![1];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_span_to_cells_in_row() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(0.0, 2.0, 0.15, 0.05);
            let expected_result = vec![0, 1];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_3_by_3_top_left() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(0.0, 2.0, 0.2, 0.2);
            let expected_result = vec![0,1,2,20,21,22,40,41,42];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_3_by_3_bottom_right() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(1.7, 0.3, 0.2, 0.2);
            let expected_result = vec![357,358,359,377,378,379,397,398,399];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_3_by_3_bottom_left() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(0.0, 0.3, 0.2, 0.2);
            let expected_result = vec![340,341,342,360,361,362,380,381,382];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_3_by_3_top_right() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(1.7, 2.0, 0.2, 0.2);
            let expected_result = vec![17,18,19,37,38,39,57,58,59];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }

        #[test]
        fn test_compute_phantom_cell_ids_middle() {
            let cell_size = 0.1;
            let bbox = cgmath::Vector4::new(1.0, 1.0, 0.2, 0.2);
            let expected_result = vec![210,211,212,230,231,232,250,251,252];
            let ss = SpatialSubdivision2D::new(cell_size);
            let result = ss.compute_phantom_cell_ids(bbox);
            assert_eq!(result, expected_result);
        }
    }

    mod test_cell_id_to_cell_type {
        use super::super::SpatialSubdivision2D;

        #[test]
        fn test_cell_id_to_cell_type_top_left() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);

            let mut cell_id = 0;
            let mut expected_result = 0;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 1;
            expected_result = 1;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 20;
            expected_result = 2;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 21;
            expected_result = 3;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);
        }

        #[test]
        fn test_cell_id_to_cell_type_top_right() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);

            let mut cell_id = 18;
            let mut expected_result = 0;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 19;
            expected_result = 1;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 38;
            expected_result = 2;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);

            cell_id = 39;
            expected_result = 3;
            assert_eq!(ss.cell_id_to_cell_type(cell_id), expected_result);
        }
    }

    mod test_construct_arrays {
        use crate::spatial_subdivision::{Object, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        use super::super::SpatialSubdivision2D;
        
        #[test]
        fn test_construct_arrays_one_object_max_num_phantom_cells(){
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id = 0;
            let phantom_cell_id_1 = 1;
            let phantom_cell_id_2 = 20;
            let phantom_cell_id_3 = 21;
            let bboxes = vec![
                cgmath::Vector4::new(-0.975, 0.975, 0.1, 0.1),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0);
            assert_eq!(object_id_array[1].id, 0);
            assert_eq!(object_id_array[2].id, 0);
            assert_eq!(object_id_array[3].id, 0);
            for i in 4..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }
            
            assert_eq!(object_id_array[0].control_bits, 0b000000);
            assert_eq!(object_id_array[1].control_bits, 0b000010);
            assert_eq!(object_id_array[2].control_bits, 0b000100);
            assert_eq!(object_id_array[3].control_bits, 0b001000);
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
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id = 379;
            let phantom_cell_id_1 = 399;
            let bboxes = vec![
                cgmath::Vector4::new(0.925, -0.825, 0.05, 0.1),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0);
            assert_eq!(object_id_array[1].id, 0);
            for i in 2..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }

            assert_eq!(object_id_array[0].control_bits, 0b010000);
            assert_eq!(object_id_array[1].control_bits, 0b011000);
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
            let ss = SpatialSubdivision2D::new(cell_size);

            let home_cell_id_1 = 20;
            let home_cell_id_2 = 21;
            let phantom_cell_id_1 = 21;
            let phantom_cell_id_2 = 22;
            let bboxes = vec![
                cgmath::Vector4::new(-0.975, 0.875, 0.1, 0.05),
                cgmath::Vector4::new(-0.875, 0.875, 0.1, 0.05),
            ];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array = vec![invalid_cell_id; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];
            let mut object_id_array = vec![Object{id: invalid_cell_id, control_bits: 0}; bboxes.len()*SPATIAL_SUBDIVISION_2D_CELL_OFFSET];

            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);

            assert_eq!(object_id_array[0].id, 0); // Home cell 1
            assert_eq!(object_id_array[1].id, 0); // Phantom cell 1
            assert_eq!(object_id_array[2].id, invalid_cell_id);
            assert_eq!(object_id_array[3].id, invalid_cell_id);
            assert_eq!(object_id_array[4].id, 1); // Home cell 2
            assert_eq!(object_id_array[5].id, 1); // Phantom cell 2
            for i in 6..object_id_array.len() {
                assert_eq!(object_id_array[i].id, invalid_cell_id);
            }

            assert_eq!(object_id_array[0].control_bits, 0b100000);
            assert_eq!(object_id_array[1].control_bits, 0b101000);
            assert_eq!(object_id_array[2].control_bits, 0);
            assert_eq!(object_id_array[3].control_bits, 0);
            assert_eq!(object_id_array[4].control_bits, 0b110000);
            assert_eq!(object_id_array[5].control_bits, 0b110100);
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

    }

    mod test_sort_arrays {
        use crate::spatial_subdivision::{Object, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        use super::super::SpatialSubdivision2D;

        #[test]
        fn test_sort_arrays() {
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);

            let bboxes = vec![
                cgmath::Vector4::new(-0.975, 0.975, 0.1, 0.1),
                cgmath::Vector4::new(-0.975, 0.975, 0.15, 0.05),
                cgmath::Vector4::new(-0.975, 0.825, 0.05, 0.1),
            ];

            let expected_cell_id_array = [0,0,1,1,20,20,21,40,u32::MAX,u32::MAX,u32::MAX,u32::MAX];
            let expeced_object_id_array = [0,1,0,1,0,2,0,2,u32::MAX,u32::MAX,u32::MAX,u32::MAX];

            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

            let object_ids = object_id_array.iter().map(|object| object.id).collect::<Vec<u32>>();

            assert_eq!(cell_id_array, expected_cell_id_array); 
            assert_eq!(object_ids, expeced_object_id_array);
        }
    }

    mod test_create_collision_cell_list {
        use crate::spatial_subdivision::{CollisionCell, Object, SpatialSubdivision2D, SPATIAL_SUBDIVISION_2D_CELL_OFFSET};

        #[test]
        fn test_create_collision_cell_list_no_collisions(){
            let cell_size = 0.1;
            let ss = SpatialSubdivision2D::new(cell_size);

            let bboxes = vec![
                cgmath::Vector4::new(-0.975, 0.975, 0.1, 0.05),
                cgmath::Vector4::new(-0.975, 0.875, 0.1, 0.05),
                cgmath::Vector4::new(-0.775, 0.975, 0.05, 0.1),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.99, 0.99, 0.01, 0.01),
                cgmath::Vector4::new(-0.94, 0.94, 0.01, 0.01),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.89, 0.99, 0.01, 0.01),
                cgmath::Vector4::new(-0.84, 0.94, 0.01, 0.01),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.99, 0.89, 0.01, 0.01),
                cgmath::Vector4::new(-0.94, 0.84, 0.01, 0.01),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.89, 0.89, 0.01, 0.01),
                cgmath::Vector4::new(-0.84, 0.84, 0.01, 0.01),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.99, 0.99, 0.01, 0.01),
                cgmath::Vector4::new(-0.925, 0.925, 0.1, 0.1),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result_pass_0 = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let expected_result_pass_1 = CollisionCell {
                offset: 2,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.99, 0.99, 0.01, 0.01),
                cgmath::Vector4::new(-0.925, 0.925, 0.1, 0.1),
                cgmath::Vector4::new(-0.88, 0.98, 0.01, 0.01),
            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
            let ss = SpatialSubdivision2D::new(cell_size);
            let expected_result_1 = CollisionCell {
                offset: 0,
                num_objects: 2,
            };
            let expected_result_2 = CollisionCell {
                offset: 2,
                num_objects: 2,
            };
            let bboxes = vec![
                cgmath::Vector4::new(-0.99, 0.89, 0.01, 0.01),
                cgmath::Vector4::new(-0.94, 0.84, 0.01, 0.01),
                cgmath::Vector4::new(-0.99, 0.69, 0.01, 0.01),
                cgmath::Vector4::new(-0.94, 0.64, 0.01, 0.01),

            ];
            let invalid_cell_id = u32::MAX;
            let mut cell_id_array: Vec<u32> = vec![invalid_cell_id;   bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
            let mut object_id_array: Vec<Object> = vec![Object{id: invalid_cell_id, control_bits: 0}.clone(); bboxes.len() * SPATIAL_SUBDIVISION_2D_CELL_OFFSET as usize];
    
            ss.constuct_arrays(bboxes, &mut cell_id_array, &mut object_id_array);
            SpatialSubdivision2D::sort_arrays(&mut cell_id_array, &mut object_id_array);

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
    }

}