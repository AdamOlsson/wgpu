

use cgmath::{InnerSpace, MetricSpace, Vector3, Vector4};

use rand::Rng;

use crate::{renderer_backend::vertex::Vertex, shapes::circle::Circle, };

use super::spatial_subdivision::SpatialSubdivision2D;


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
        let radius = 0.01;
        let num_instances: u32 = 4000;
                
        let positions = CollisionSimulation::generate_initial_positions(num_instances, radius);
        // let mut positions = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
        // positions[0] = Vector3::new(-0.5, 0.0, 0.0);
        // positions[1] = Vector3::new(0.5, 0.0, 0.0);
        
        let velocities = CollisionSimulation::generate_random_velocities();
        // let mut velocities = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
        // velocities[0] = Vector3::new(0.01, 0.0, 0.0);
        // velocities[1] = Vector3::new(-0.01, 0.0, 0.0);

        let colors = CollisionSimulation::genrate_random_colors();
        let mass = [1.0; MAX_INSTANCES];
        let vertices = Circle::compute_vertices([0.0, 0.0, 0.0], radius);
        let indices = Circle::compute_indices();
        let num_indices = (359)*3;

        Self { positions, colors, mass, velocities, num_instances, radius, indices, num_indices, vertices }
    }

    /// Computes the response of a collision between two circles.
    /// 
    /// Args:
    /// - ttc: Time to collision
    /// - va: Velocity of circle A
    /// - vb: Velocity of circle B
    /// - pa: Position of circle A
    /// - pb: Position of circle B
    /// - ma: Mass of circle A
    /// - mb: Mass of circle B
    /// 
    /// The function computes the new velocities of the circles after a collision.
    fn circle_circle_collision_response(
            ttc: f32, 
            va: &mut Vector3<f32>, vb: &mut Vector3<f32>,
            pa: &mut Vector3<f32>, pb: &mut Vector3<f32>,
            ma: f32, mb: f32) {
        let collision_point_a = *pa + ttc * (*va);
        let collision_point_other = *pb + ttc * (*vb);
        let normal = (collision_point_a - collision_point_other).normalize();
        let p = 2.0 * (va.dot(normal) - vb.dot(normal)) / (ma + mb);
        let new_velocity_i = *va - p * mb * normal;
        let new_velocity_j = *vb + p * ma * normal;
        *va = new_velocity_i;
        *vb = new_velocity_j;
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
            CollisionSimulation::continous_circle_circle_collision_detection,
            CollisionSimulation::circle_circle_collision_response,
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

    /// Axis-aligned bounding box collision detection.
    /// 
    /// Axis-aligned bounding box narrow collision detection algorithm that
    /// can be used to determine if two 2D shapes are intersecting. The algorithm
    /// works by projecting the shapes onto a set of axes and checking if the projections
    /// overlap. If the projections overlap, the shapes are intersecting.
    /// 
    /// To create a linear collision detection we use the start and end position of the
    /// objects and create a bounding bounding box. We then project the bounding box onto
    /// the axises and check if the projections overlap.
    #[allow(dead_code)]
    fn aabb_collision_detection(
            pos: Vector3<f32>, new_pos: Vector3<f32>,
            pos_other: Vector3<f32>, new_pos_other: Vector3<f32>,
            radius: f32) -> bool{

        // Check x-axis
        let x1_start = pos[0].min(new_pos[0]) - radius;
        let x1_end = pos[0].max(new_pos[0]) + radius;
        let x2_start = pos_other[0].min(new_pos_other[0]) - radius;
        let x2_end = pos_other[0].max(new_pos_other[0]) + radius;
        
        if x1_end < x2_start || x1_start > x2_end {
            return false;
        }

        // Check y-axis
        let y1_start = pos[1].min(new_pos[1]) - radius;
        let y1_end = pos[1].max(new_pos[1]) + radius;
        let y2_start = pos_other[1].min(new_pos_other[1]) - radius;
        let y2_end = pos_other[1].max(new_pos_other[1]) + radius;

        if y1_end < y2_start || y1_start > y2_end {
            return false;
        }

        true
    }
    
    /// Computes the time of collision between two circles.
    /// 
    /// Returns a value between 0.0 and 1.0 representing the fraction of a timestep
    /// until the collision returns. If there is no collision, None is returned.
    fn continous_circle_circle_collision_detection(
            circle_1_old_pos: Vector3<f32>, circle_1_new_pos: Vector3<f32>,
            radius_circle_1:f32, circle_2_old_pos: Vector3<f32>,
            circle_2_new_pos: Vector3<f32>, radius_circle_2: f32) -> Option<f32> {
        let h_old_pos = circle_1_old_pos;
        let h_new_pos = circle_1_new_pos;
        let g_old_pos = circle_2_old_pos;
        let g_new_pos = circle_2_new_pos;

        let delta_h = h_new_pos - h_old_pos;
        let delta_g = g_new_pos - g_old_pos;
        let a = delta_h.dot(delta_h).abs() + delta_g.dot(delta_g).abs() + 2.0 * delta_h.dot(delta_g).abs();

        if a == 0.0 {
            // The circles are not moving
            return None;
        }

        let b = 2.0 * (delta_h.dot(h_old_pos) + delta_g.dot(g_old_pos) - delta_h.dot(g_old_pos) - delta_g.dot(h_old_pos));
        let c = h_old_pos.dot(h_old_pos) + g_old_pos.dot(g_old_pos) - 2.0 * h_old_pos.dot(g_old_pos) - (radius_circle_1 + radius_circle_2).powi(2);

        let discriminant = b.powi(2) - 4.0 * a * c;

        if discriminant < 0.0 {
            // There exists no solition to the equation
            return None;
        }
        let t1 = (-b + f32::sqrt(discriminant)) / (2.0 * a);
        let t2 = (-b - f32::sqrt(discriminant)) / (2.0 * a);
        if t1 < 0.0 && t2 < 0.0 {
            // The collision happened in the past
            return None;
        }

        if t1 >= 0.0 && t2 >= 0.0 {
            // The collision will happen in the future, return the smallest value
            // as the smallar is the time of collision and the larger when the circles
            // would exit each other if the would pass through.
            return Some(t1.min(t2));
        }

        // Finally, either t1 or t2 is positive. I am unsure what to return in this case
        // as this means that we already are in a collision. For now I return the negative
        // value to signal that we are in a collision.
        return Some(t1.min(t2));
    }

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
        CollisionSimulation::boundary_collision(BOTTOM_LEFT, TOP_LEFT,
            curr_pos, new_pos, radius); // Left boundary
        match res_left {
            None => match CollisionSimulation::boundary_collision(TOP_RIGHT, BOTTOM_RIGHT,
                curr_pos, new_pos, radius) { // Right boundary
                None => return None, // No collision
                Some(line_collision_point) =>
                    Some(CollisionSimulation::compute_collision_point(
                        TOP_RIGHT, BOTTOM_RIGHT, 
                        curr_pos, velocity,
                        line_collision_point, radius)), // Collision with right boundary
            }, 
            Some(line_collision_point) => 
                Some(CollisionSimulation::compute_collision_point(
                    BOTTOM_LEFT, TOP_LEFT, 
                    curr_pos, velocity,
                    line_collision_point, radius)),  // Collision with left boundary,
        }
    }

    fn horizontal_boundary_collision(
            curr_pos: Vector3<f32>, new_pos: Vector3<f32>,
            velocity: Vector3<f32>, radius: f32) -> Option<Vector3<f32>> {

        let res_bottom =
            CollisionSimulation::boundary_collision( BOTTOM_LEFT, BOTTOM_RIGHT,
                curr_pos, new_pos, radius);
        match res_bottom {
            None => match CollisionSimulation::boundary_collision(TOP_LEFT, TOP_RIGHT,
                    curr_pos, new_pos, radius) {
                None => return None, // No collision
                Some(line_collision_point) =>
                Some(CollisionSimulation::compute_collision_point(
                    TOP_LEFT, TOP_RIGHT, 
                    curr_pos, velocity,
                    line_collision_point, radius)), // Collision with top boundary
            },
            Some(line_collision_point) =>
                Some(CollisionSimulation::compute_collision_point(
                    BOTTOM_LEFT, BOTTOM_RIGHT, 
                    curr_pos, velocity,
                    line_collision_point, radius)) // Collision with bottom boundary
            
        }
    }

    /// Computes the point where the circle will collide with the boundary.
    /// 
    /// The function assumes that the circle will collide with the boundary in the next
    /// timestep. It computes the collision point with respect to the circle's radius.
    /// 
    /// Returns the point where the circle will collide with the boundary.
    fn compute_collision_point(
                boundary_start: Vector3<f32>, boundary_stop: Vector3<f32>,
                curr_position: Vector3<f32>, velocity_vec: Vector3<f32>,
                line_collision_point: Vector3<f32>, radius: f32) -> Vector3<f32>{
        let a = line_collision_point;
        let r = radius;
        let c = curr_position;
        let a_c_len = (c).distance(a); // Can be optimized by not taking sqrt (.distance2())
        let p1 = CollisionSimulation::closest_point_on_line(
            boundary_start, boundary_stop, 
            Vector3::new(c[0], c[1], c[2]));
        let p1_c = (c).distance(p1); // Can be optimized by not taking sqrt (.distance2())
        let v = f32::sqrt(velocity_vec[0].powi(2) + velocity_vec[1].powi(2));
        let p2_x = a[0] - r * (a_c_len/p1_c)*velocity_vec[0]/v;
        let p2_y = a[1] - r * (a_c_len/p1_c)*velocity_vec[1]/v;

        Vector3::new(p2_x, p2_y, 0.0)
    }

    /// Evaluates wether a circle is about to collide with a boundary. It does so by 
    /// checking if the line of the velocity direction intersects with the boundary.
    /// If the lines intersect, there is a collision if:
    /// - The distance between the collision point and the new position is less than the radius
    /// - The distance between the closest point on the boundary and the new position is less than the radius
    /// 
    /// Returns the point where the velocity line intersects with the boundary if there is a collision.
    fn boundary_collision(boundary_start: Vector3<f32>, boundary_stop: Vector3<f32>,
            curr_position: Vector3<f32>, new_position: Vector3<f32>, radius: f32) -> Option<Vector3<f32>> {
        match CollisionSimulation::line_line_intersect(boundary_start.into(), boundary_stop.into(), curr_position.into(), new_position.into()) {
            None => return None, // No collision
            Some(collision_point) => {
                // TODO: There are additional checks that can be done here to make sure there is a collision
                let distance1 = collision_point.distance2(new_position);
                if distance1 < radius.powi(2) {
                    return Some(collision_point);
                }

                let distance2 =
                    CollisionSimulation::closest_point_on_line(boundary_start, boundary_stop, new_position)
                    .distance2(new_position);
                if distance2 < radius.powi(2) {
                    return Some(collision_point);
                }

                return None;
            },
        }
    }

    /// Computes the intersection point of two lines.
    /// 
    /// Returns the intersection point if the lines intersect, otherwise None.
    #[allow(non_snake_case)]
    fn line_line_intersect(A: [f32;3], B: [f32;3], C: [f32;3], D: [f32;3]) -> Option<Vector3<f32>>{
        let a1 = B[1] - A[1];
        let b1 = A[0] - B[0];
        let c1 = a1 * A[0] + b1 * A[1];

        let a2 = D[1] - C[1];
        let b2 = C[0] - D[0];
        let c2 = a2 * C[0] + b2 * C[1];

        let determinant = a1 * b2 - a2 * b1;
        if determinant == 0.0 {
            // The lines are parallel
            return None;
        }
        let x = (b2 * c1 - b1 * c2) / determinant;
        let y = (a1 * c2 - a2 * c1) / determinant;
        Some(Vector3::new(x, y, 0.0))
    }

    /// Computes the closest point on a line to a given point.
    /// 
    /// Returns the closest point on the line.
    fn closest_point_on_line(line_start: Vector3<f32>, line_end: Vector3<f32>, p: Vector3<f32>) -> Vector3<f32> {
        // y = ax + b
        let ab = line_end - line_start;
        let t = (p-line_start).dot(ab) / ab.dot(ab);
        let proj = line_start + t * ab;
        return proj;

    }

    fn generate_random_velocities() -> [Vector3<f32>; MAX_INSTANCES] {
        let mut velocities = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
        let mut rng = rand::thread_rng();
        for i in 0..MAX_INSTANCES {
            velocities[i] = Vector3::new(rng.gen_range(-0.01..=0.01), rng.gen_range(-0.01..=0.01), 0.0);
        }
        velocities
    }

    fn genrate_random_colors() -> [Vector3<f32>; MAX_INSTANCES] {
        let mut colors = [Vector3::new(0.0, 0.0, 0.0); MAX_INSTANCES];
        let mut rng = rand::thread_rng();
        let min = 0.2;
        let max = 1.0;
        for i in 0..MAX_INSTANCES {
            colors[i] = Vector3::new(rng.gen_range(min..max), rng.gen_range(min..max), rng.gen_range(min..max));
        }
        colors
    }

    fn generate_initial_positions(num_instances: u32, radius: f32) -> [Vector3<f32>; MAX_INSTANCES]{
        let grid_side = f32::round(f32::sqrt(num_instances as f32) + 1.0);
        if grid_side * radius >= 2.0 {
            panic!("Radius too large for number of instances.");
        }

        let delta = 2.0 / grid_side;
        let mut positions = [Vector3::new(0.0,0.0,0.0); MAX_INSTANCES];
        for i in 0..grid_side as usize {
            for j in 0..grid_side as usize {
                positions[i * grid_side as usize + j] = Vector3::new(
                    delta * i as f32 - 1.0 + delta / 2.0,
                    delta * j as f32 - 1.0 + delta / 2.0,
                    0.0,
                );
            }
        }
        positions
    }
}


#[cfg(test)]
mod tests {
    use cgmath::Vector3;

    #[test]
    fn test_closest_point_on_line_horizontal_bottom() {
        let line_start = Vector3::new(-1.0, -1.0, 0.0);
        let line_end = Vector3::new(1.0, -1.0, 0.0);
        let p = Vector3::new(0.0, 0.0, 0.0);
        let expected_result = Vector3::new(0.0, -1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_horizontal_bottom_negative_x() {
        let line_start = Vector3::new(-1.0, -1.0, 0.0);
        let line_end = Vector3::new(1.0, -1.0, 0.0);
        let p = Vector3::new(-0.5, 0.0, 0.0);
        let expected_result = Vector3::new(-0.5, -1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_horizontal_bottom_positive_x() {
        let line_start = Vector3::new(-1.0, -1.0, 0.0);
        let line_end = Vector3::new(1.0, -1.0, 0.0);
        let p = Vector3::new(0.5, 0.0, 0.0);
        let expected_result = Vector3::new(0.5, -1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_horizontal_top() {
        let line_start = Vector3::new(-1.0, 1.0, 0.0);
        let line_end = Vector3::new(1.0, 1.0, 0.0);
        let p = Vector3::new(0.0, 0.0, 0.0);
        let expected_result = Vector3::new(0.0, 1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_horizontal_top_negative_x() {
        let line_start = Vector3::new(-1.0, 1.0, 0.0);
        let line_end = Vector3::new(1.0, 1.0, 0.0);
        let p = Vector3::new(-0.5, 0.0, 0.0);
        let expected_result = Vector3::new(-0.5, 1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_horizontal_top_positive_x() {
        let line_start = Vector3::new(-1.0, 1.0, 0.0);
        let line_end = Vector3::new(1.0, 1.0, 0.0);
        let p = Vector3::new(0.5, 0.0, 0.0);
        let expected_result = Vector3::new(0.5, 1.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_45_degrees() {
        let line_start = Vector3::new(-1.0, -1.0, 0.0);
        let line_end = Vector3::new(1.0, 1.0, 0.0);
        let p = Vector3::new(-0.5, 0.5, 0.0);
        let expected_result = Vector3::new(0.0, 0.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_closest_point_on_line_p_on_line() {
        let line_start = Vector3::new(-1.0, -1.0, 0.0);
        let line_end = Vector3::new(1.0, 1.0, 0.0);
        let p = Vector3::new(0.0, 0.0, 0.0);
        let expected_result = Vector3::new(0.0, 0.0, 0.0);
        let res = super::CollisionSimulation::closest_point_on_line(line_start, line_end, p);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_continious_circle_collision_no_collision() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(0.2, 0.0, 0.0);
        let circle_1_new_pos = Vector3::new(0.3, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(-0.2, 0.0, 0.0);
        let circle_2_new_pos = Vector3::new(-0.3, 0.0, 0.0);
        let expected_result: Option<f32> = None;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_continious_circle_collision_static_dynamic_circles1() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(0.0, 0.0, 0.0);
        let circle_1_new_pos = Vector3::new(0.0, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.25, 0.0, 0.0);
        let circle_2_new_pos = Vector3::new(0.15, 0.0, 0.0);
        let expected_result = 0.5;
        let epsilon: f32 = 0.0001;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_ne!(res, None, "Expected collision but got None");
        let diff = res.unwrap() - expected_result;
        assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_static_dynamic_circles2() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(0.25, 0.0, 0.0);
        let circle_1_new_pos = Vector3::new(0.15, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.0, 0.0, 0.0);
        let circle_2_new_pos = Vector3::new(0.0, 0.0, 0.0);
        let expected_result = 0.5;
        let epsilon: f32 = 0.0001;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_ne!(res, None, "Expected collision but got None");
        let diff = res.unwrap() - expected_result;
        assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_static_static_circles_no_collision() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(-0.15, 0.0, 0.0);
        let circle_1_new_pos = Vector3::new(-0.15, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.15, 0.0, 0.0);
        let circle_2_new_pos = Vector3::new(0.15, 0.0, 0.0);
        let expected_result: Option<f32> = None;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn test_continious_circle_collision_dynamic_dynamic_circles_x() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(-0.15, 0.0, 0.0);
        let circle_1_new_pos = Vector3::new(-0.05, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.15, 0.0, 0.0);
        let circle_2_new_pos = Vector3::new(0.05, 0.0, 0.0);
        let expected_result = 0.5;
        let epsilon: f32 = 0.0001;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_ne!(res, None, "Expected collision but got None");
        let diff = res.unwrap() - expected_result;
        assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_dynamic_dynamic_circles_y() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(0.0, -0.15, 0.0);
        let circle_1_new_pos = Vector3::new(0.0, -0.05, 0.0);
        let circle_2_old_pos = Vector3::new(0.0, 0.15, 0.0);
        let circle_2_new_pos = Vector3::new(0.0, 0.05, 0.0);
        let expected_result = 0.5;
        let epsilon: f32 = 0.0001;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_ne!(res, None, "Expected collision but got None");
        let diff = res.unwrap() - expected_result;
        assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_dynamic_dynamic_circles_different_velocities_collide() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(0.0, -0.15, 0.0);
        let circle_1_new_pos = Vector3::new(0.0, -0.05, 0.0);
        let circle_2_old_pos = Vector3::new(0.0, 0.2, 0.0);
        let circle_2_new_pos = Vector3::new(0.0, 0.0, 0.0);
        let expected_result = 0.5;
        let epsilon: f32 = 0.0001;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_ne!(res, None, "Expected collision but got None");
        let diff = res.unwrap() - expected_result;
        assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_dynamic_dynamic_circles_no_collision() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(-0.5, -0.2, 0.0);
        let circle_1_new_pos = Vector3::new(-0.5, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.5, 0.2, 0.0);
        let circle_2_new_pos = Vector3::new(0.5, 0.0, 0.0);
        let expected_result = None;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

    #[test]
    fn test_continious_circle_collision_dynamic_dynamic_circles_no_collision_2() {
        let radius = 0.1;
        let circle_1_old_pos = Vector3::new(-0.5, 0.2, 0.0);
        let circle_1_new_pos = Vector3::new(-0.5, 0.0, 0.0);
        let circle_2_old_pos = Vector3::new(0.5, 0.2, 0.0);
        let circle_2_new_pos = Vector3::new(0.5, 0.0, 0.0);
        let expected_result = None;
        let res = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res.unwrap());
    }

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
        
        let res1 = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_a_prev_pos, circle_a_next_pos, radius, circle_b_prev_pos, circle_b_next_pos, radius);
        assert_ne!(res1, None, "Expected collision but got None");
        
        let res2 = super::CollisionSimulation::continous_circle_circle_collision_detection(
            circle_a_prev_pos, circle_a_curr_pos, radius, circle_b_prev_pos, circle_b_curr_pos, radius);
        assert_ne!(res2, None, "Expected collision but got None");
        
        let ttc = res1.unwrap();
        super::CollisionSimulation::circle_circle_collision_response(
            ttc, &mut circle_a_prev_velocity, &mut circle_b_prev_velocity, 
            &mut circle_a_prev_pos, &mut circle_b_prev_pos, 1.0,
            1.0 );
        
        // Validate that the response would handle the scenario correctly
        assert_ne!(circle_a_prev_velocity, circle_a_curr_velocity, "Expected different velocities but got the same");
        assert_ne!(circle_b_prev_velocity, circle_b_curr_velocity, "Expected different velocities but got the same");
    
    }

    #[test]
    fn aabb_collision_detection_x_axis_separated() {
        let radius = 0.1;
        let pos = Vector3::new(-0.2, 0.0, 0.0);
        let new_pos = Vector3::new(-0.1, 0.0, 0.0);
        let pos_other = Vector3::new(0.2, 0.0, 0.0);
        let new_pos_other = Vector3::new(0.3, 0.0, 0.0);
        let expected_result = false;
        let res = super::CollisionSimulation::aabb_collision_detection(
            pos, new_pos, pos_other, new_pos_other, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn aabb_collision_detection_x_axis_collision() {
        let radius = 0.1;
        let pos = Vector3::new(0.0, 0.0, 0.0);
        let new_pos = Vector3::new(0.1, 0.0, 0.0);
        let pos_other = Vector3::new(0.2, 0.0, 0.0);
        let new_pos_other = Vector3::new(0.3, 0.0, 0.0);
        let expected_result = true;
        let res = super::CollisionSimulation::aabb_collision_detection(
            pos, new_pos, pos_other, new_pos_other, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn aabb_collision_detection_y_axis_separated() {
        let radius = 0.1;
        let pos = Vector3::new(0.0, -0.2, 0.0);
        let new_pos = Vector3::new(0.0, -0.1, 0.0);
        let pos_other = Vector3::new(0.0, 0.2, 0.0);
        let new_pos_other = Vector3::new(0.0, 0.3, 0.0);
        let expected_result = false;
        let res = super::CollisionSimulation::aabb_collision_detection(
            pos, new_pos, pos_other, new_pos_other, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
    }

    #[test]
    fn aabb_collision_detection_y_axis_collision() {
        let radius = 0.1;
        let pos = Vector3::new(0.0, 0.0, 0.0);
        let new_pos = Vector3::new(0.0, 0.1, 0.0);
        let pos_other = Vector3::new(0.0, 0.2, 0.0);
        let new_pos_other = Vector3::new(0.0, 0.3, 0.0);
        let expected_result = true;
        let res = super::CollisionSimulation::aabb_collision_detection(
            pos, new_pos, pos_other, new_pos_other, radius);
        assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);        
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