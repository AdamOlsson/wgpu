

use cgmath::{InnerSpace, Vector3, Vector4};

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle, };

use super::{spatial_subdivision::SpatialSubdivision2D, util::{collision_detection::{self, continous_circle_circle_collision_detection}, collision_response::circle_circle_collision_response, generate_initial_positions_square_grid, generate_random_colors, generate_random_velocities}};


const MAX_INSTANCES: usize = 10000;

#[derive(PartialEq)]
enum Boundary {
    _Top,
    _Bottom,
    _Left,
    _Right,
    Horizontal,
    Vertical,
}

pub struct CollisionSimulation {
    pub positions: [Vector3<f32>; MAX_INSTANCES],
    pub colors: [Vector3<f32>; MAX_INSTANCES],
    pub mass: [f32; MAX_INSTANCES],
    velocities: [Vector3<f32>; MAX_INSTANCES],

    pub num_instances: u32,
    radius: f32,

    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
}

const TOP_LEFT: Vector3<f32> = Vector3::new(-1.0, 1.0, 0.0);
const TOP_RIGHT: Vector3<f32> = Vector3::new(1.0, 1.0, 0.0);
const BOTTOM_LEFT: Vector3<f32> = Vector3::new(-1.0, -1.0, 0.0);
const BOTTOM_RIGHT: Vector3<f32> = Vector3::new(1.0, -1.0, 0.0);

impl CollisionSimulation {
    pub fn new() -> Self{
        let radius = 0.1;
        let num_instances: u32 = 20;
                
        let positions = generate_initial_positions_square_grid(num_instances, radius);        
        let velocities = generate_random_velocities();

        let colors = generate_random_colors();
        let mass = [1.0; MAX_INSTANCES];
        let vertices = Circle::compute_vertices([0.0, 0.0, 0.0], 1.0);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;

        Self { positions, colors, mass, velocities, num_instances, radius, indices, num_indices, vertices }
    }

    fn create_bbox(pos: Vector3<f32>, vel: Vector3<f32>, radius: f32) -> Vector4<f32> {
        let new_pos = pos + vel;
        let top_left_x = pos[0].min(new_pos[0]) - radius;
        let top_left_y = pos[1].max(new_pos[1]) + radius;
        let bot_right_x = pos[0].max(new_pos[0]) + radius;
        let bot_left_y = pos[1].min(new_pos[1]) - radius;
        let width = (bot_right_x - top_left_x).abs();
        let height = (top_left_y - bot_left_y).abs();
        Vector4::new(top_left_x, top_left_y, width, height)
    }

    fn create_bounding_sphere(pos: Vector3<f32>, vel: Vector3<f32>, r: f32) -> Vector3<f32> {
        let new_pos = pos + vel;
        let center = (pos + new_pos) / 2.0;
        let radius = (pos - center).magnitude();
        let sphere_total_radius = (radius + r) * f32::sqrt(2.0); // Scale sphere as described
        Vector3::new(center[0], center[1], sphere_total_radius)
    }

    pub fn update(&mut self) {
        // Define bounding spheres and the grid cell size
        let mut bounding_spheres = Vec::new();
        let mut max_bounding_radius = 0.0;
        for i in 0..self.num_instances as usize {
            let bounding_sphere = CollisionSimulation::create_bounding_sphere(self.positions[i], self.velocities[i], self.radius);
            let radius = bounding_sphere[2];
            
            if radius > max_bounding_radius {
                max_bounding_radius = radius;
            }
            bounding_spheres.push(bounding_sphere);
        }



        let cell_size = max_bounding_radius * 2.0 * 1.5;
        let ss = SpatialSubdivision2D::new(cell_size);
        ss.run(&mut self.positions, &mut self.velocities,
            continous_circle_circle_collision_detection,
            circle_circle_collision_response,
            self.radius, &self.mass, &bounding_spheres);

        
        for i in 0..self.num_instances as usize {
            let pos = self.positions[i];
            let new_pos = pos + self.velocities[i];            

            match CollisionSimulation::vertical_boundary_collision(pos, new_pos, self.velocities[i], self.radius) {
                None => (), // No collision
                Some(collision_point) => 
                    CollisionSimulation::boundary_collision_response(
                        pos, new_pos, collision_point, &mut self.velocities[i], Boundary::Vertical),
            }

            match CollisionSimulation::horizontal_boundary_collision(pos, new_pos, self.velocities[i], self.radius) {
                None => (), // No collision
                Some(collision_point) => 
                    CollisionSimulation::boundary_collision_response(
                        pos, new_pos, collision_point, &mut self.velocities[i], Boundary::Horizontal),
            }

            self.positions[i] += self.velocities[i];

        }
    }

    // TODO: implement this function so that we in the future can compare
    // the naive collision detection with the spatial subdivision algorithm.
    // fn naive_collision_detection() {
    //     for j in i..self.num_instances as usize {
    //         if i == j {
    //             continue;
    //         }

    //         let pos_other = self.positions[j];
    //         let new_pos_other = pos_other + self.velocities[j];

    //         if !CollisionSimulation::aabb_collision_detection(
    //                 pos, new_pos, self.positions[j], self.positions[j] + self.velocities[j], self.radius) {
    //             continue;
    //         }


    //         match CollisionSimulation::continous_circle_circle_collision_detection(
    //             pos, new_pos, self.radius, pos_other, new_pos_other, self.radius) {
    //             None => (), // No collision
    //             Some(t) if -1.0 <= t && t <= 1.0 => {
    //                 let mut va = self.velocities[i].clone();
    //                 let mut vb = self.velocities[j].clone();
    //                 let mut pa = self.positions[i].clone();
    //                 let mut pb = self.positions[j].clone();
                    
    //                 CollisionSimulation::circle_circle_collision_response(
    //                     t, &mut va, &mut vb, &mut pa, &mut pb,
    //                     self.mass[i], self.mass[j]);

    //                 self.velocities[i] = va;
    //                 self.velocities[j] = vb;
    //             },                    
    //             _ => (),
    //         }
    //     }
    // }


    /// Computes the new position of the circle after a collision with a boundary.
    ///
    /// Note: This function will change the velocity of the circle.
    fn boundary_collision_response(
            _curr_pos: Vector3<f32>, _new_pos: Vector3<f32>,
            _collision_point: Vector3<f32>, velocity: &mut Vector3<f32>, boundary: Boundary) {
        // We do not care to compute the exact position after the collision, only the new velocity
        // let travel_distance = curr_pos.distance2(new_pos);
        // let collision_distance = curr_pos.distance2(collision_point);
        // let travel_distance_after_collision = travel_distance - collision_distance;
        // let travel_distance_after_collision_percent = travel_distance_after_collision / travel_distance;
        
        if boundary == Boundary::Horizontal {
            velocity[1] = -velocity[1];
        } else if boundary == Boundary::Vertical {
            velocity[0] = -velocity[0];
        } else {
            panic!("Invalid boundary type. Valid types are Horizontal and Vertical");
        }
        
        // let position_after_collision = collision_point + (*velocity)*travel_distance_after_collision_percent;
        // position_after_collision

    }

    fn vertical_boundary_collision(
            curr_pos: Vector3<f32>, new_pos: Vector3<f32>,
            velocity: Vector3<f32>, radius: f32) -> Option<Vector3<f32>> {

        let res_left =
        collision_detection::circle_line_collision_detection(BOTTOM_LEFT, TOP_LEFT,
            curr_pos, new_pos, radius); // Left boundary
        match res_left {
            None => match collision_detection::circle_line_collision_detection(TOP_RIGHT, BOTTOM_RIGHT,
                curr_pos, new_pos, radius) { // Right boundary
                None => return None, // No collision
                Some(line_collision_point) =>
                    Some(collision_detection::compute_collision_point(
                        TOP_RIGHT, BOTTOM_RIGHT, 
                        curr_pos, velocity,
                        line_collision_point, radius)), // Collision with right boundary
            }, 
            Some(line_collision_point) => 
                Some(collision_detection::compute_collision_point(
                    BOTTOM_LEFT, TOP_LEFT, 
                    curr_pos, velocity,
                    line_collision_point, radius)),  // Collision with left boundary,
        }
    }

    fn horizontal_boundary_collision(
            curr_pos: Vector3<f32>, new_pos: Vector3<f32>,
            velocity: Vector3<f32>, radius: f32) -> Option<Vector3<f32>> {

        let res_bottom =
        collision_detection::circle_line_collision_detection( BOTTOM_LEFT, BOTTOM_RIGHT,
                curr_pos, new_pos, radius);
        match res_bottom {
            None => match collision_detection::circle_line_collision_detection(TOP_LEFT, TOP_RIGHT,
                    curr_pos, new_pos, radius) {
                None => return None, // No collision
                Some(line_collision_point) =>
                Some(collision_detection::compute_collision_point(
                    TOP_LEFT, TOP_RIGHT, 
                    curr_pos, velocity,
                    line_collision_point, radius)), // Collision with top boundary
            },
            Some(line_collision_point) =>
                Some(collision_detection::compute_collision_point(
                    BOTTOM_LEFT, BOTTOM_RIGHT, 
                    curr_pos, velocity,
                    line_collision_point, radius)) // Collision with bottom boundary
            
        }
    }
}


#[cfg(test)]
mod tests {
    use cgmath::Vector3;

    use crate::simulation::util::{collision_detection::continous_circle_circle_collision_detection, collision_response::circle_circle_collision_response};

    #[test]
    fn test_continious_circle_collision_live_scenario_4() {
        let radius = 0.1;
        let mut circle_a_prev_pos = Vector3::new(-0.013462657, 0.06742557,  0.0);
        let circle_a_curr_pos = Vector3::new(-0.0186382,   0.076581195, 0.0);
        let circle_a_next_pos = Vector3::new(-0.023813741, 0.08573682,  0.0);
        let mut circle_a_prev_velocity: Vector3<f32> = Vector3::new(-0.0051755426, 0.009155621, 0.0);
        let circle_a_curr_velocity = Vector3::new(-0.0051755426, 0.009155621, 0.0);
        
        let mut circle_b_prev_pos = Vector3::new(-0.056902945, 0.2725812,  0.0);
        let circle_b_curr_pos = Vector3::new(-0.056167364, 0.26809716, 0.0);
        let circle_b_next_pos = Vector3::new(-0.055431783, 0.26361313, 0.0);
        let mut circle_b_prev_velocity: Vector3<f32> = Vector3::new(0.0007355814, -0.0044840397, 0.0);
        let circle_b_curr_velocity = Vector3::new(0.0007355814, -0.0044840397, 0.0);
        
        let res1 = continous_circle_circle_collision_detection(
            &circle_a_prev_pos, &circle_a_next_pos, radius, &circle_b_prev_pos, &circle_b_next_pos, radius);
        assert_ne!(res1, None, "Expected collision but got None");
        
        let res2 = continous_circle_circle_collision_detection(
            &circle_a_prev_pos, &circle_a_curr_pos, radius, &circle_b_prev_pos, &circle_b_curr_pos, radius);
        assert_ne!(res2, None, "Expected collision but got None");
        
        let ttc = res1.unwrap();
        circle_circle_collision_response(
            ttc, &mut circle_a_prev_velocity, &mut circle_b_prev_velocity, 
            &mut circle_a_prev_pos, &mut circle_b_prev_pos, 1.0,
            1.0, 1.0 );
        
        // Validate that the response would handle the scenario correctly
        assert_ne!(circle_a_prev_velocity, circle_a_curr_velocity, "Expected different velocities but got the same");
        assert_ne!(circle_b_prev_velocity, circle_b_curr_velocity, "Expected different velocities but got the same");
    
    }

    

    mod test_create_bounding_sphere {
        use cgmath::Vector3;
        use super::super::CollisionSimulation;

        #[test]
        fn create_bounding_sphere_no_velocity() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, 0.0, 0.0);
            let velocity = Vector3::new(0.0, 0.0, 0.0);
            
            let expected_result = Vector3::new(0.0, 0.0, radius*f32::sqrt(2.0));
            let res = CollisionSimulation::create_bounding_sphere(pos, velocity, radius);

            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn create_bounding_sphere_with_velocity() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, 0.0, 0.0);
            let velocity = Vector3::new(0.2, 0.0, 0.0);
            
            let expected_result = Vector3::new(0.1, 0.0, 0.2*f32::sqrt(2.0));
            let res = CollisionSimulation::create_bounding_sphere(pos, velocity, radius);

            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn create_bounding_sphere_with_negative_velocity() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, 0.0, 0.0);
            let velocity = Vector3::new(-0.2, 0.0, 0.0);
            
            let expected_result = Vector3::new(-0.1, 0.0, 0.2*f32::sqrt(2.0));
            let res = CollisionSimulation::create_bounding_sphere(pos, velocity, radius);

            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
    }

}