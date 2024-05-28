use cgmath::{InnerSpace, MetricSpace, Vector3};

/// Continous dynamic circle and static line collision detection.
/// 
/// Evaluates wether a circle is about to collide with a line. It does so by 
/// checking if the line of the velocity direction intersects with the line.
/// If the lines intersect, there is a collision if:
/// - The distance between the collision point and the new position is less than the radius
/// - The distance between the closest point on the boundary and the new position is less than the radius
/// 
/// Returns the point where the velocity line intersects with the boundary if there is a collision.
pub(crate) fn circle_line_collision_detection(line_start: Vector3<f32>, line_stop: Vector3<f32>,
        curr_position: Vector3<f32>, new_position: Vector3<f32>, radius: f32) -> Option<Vector3<f32>> {
    match do_lines_intersect(line_start.into(), line_stop.into(), curr_position.into(), new_position.into()) {
        None => return None, // No collision
        Some(collision_point) => {
            // TODO: There are additional checks that can be done here to make sure there is a collision
            // https://ericleong.me/research/circle-line/
            let distance1 = collision_point.distance2(new_position);
            if distance1 < radius.powi(2) {
                return Some(collision_point);
            }

            let distance2 =
                closest_point_on_line(line_start, line_stop, new_position)
                .distance2(new_position);
            if distance2 < radius.powi(2) {
                return Some(collision_point);
            }

            return None;
        },
    }
}

/// Evaluate if a circle will collide with a line segment in the next timestep
/// 
/// Returns the fraction of a timestep until a collision occurs, otherwise none.
pub(crate) fn circle_line_segment_collision_detection(
        line_start: Vector3<f32>, line_end: Vector3<f32>, 
        curr_pos: Vector3<f32>, new_pos: Vector3<f32>,
        radius: f32) -> Option<f32> {

    // Intersect point is on the movement vector
    let intersect_point = do_line_segments_intersect(
        line_start, line_end, curr_pos, new_pos);
    if intersect_point != None {
        let ttc = continous_circle_circle_collision_detection(
            curr_pos, new_pos, radius, intersect_point.unwrap(), intersect_point.unwrap(), 0.0);
        return ttc;
    }

    let radius_sqrd = radius.powi(2);
    let point_b = closest_point_on_line_segment(line_start, line_end, new_pos);
    // b is less than r away from the movement end and on the segment
    if point_b.distance2(new_pos) < radius_sqrd {
        let ttc = continous_circle_circle_collision_detection(
            curr_pos, new_pos, radius, point_b, point_b, 0.0);
        return ttc;
    }

    let point_c = closest_point_on_line_segment(curr_pos,new_pos, line_start);
    // c is less than r away from line start and on the movement vector
    if point_c.distance2(line_start) < radius_sqrd {
        // We treat ends of line segments as circle with radius 0
        let ttc = continous_circle_circle_collision_detection(
            curr_pos, new_pos, radius,
            line_start, line_start, 0.0);
        return ttc;
    }

    let point_d = closest_point_on_line_segment(curr_pos,new_pos, line_end);
    // d is less than r away from line start and on the movement vector
    if point_d.distance2(line_end) < radius_sqrd {
        // We treat ends of line segments as circle with radius 0
        let ttc = continous_circle_circle_collision_detection(
            curr_pos, new_pos, radius,
            line_end, line_end, 0.0);
        return ttc;
    }
    return None;
}

/// Computes the intersection point of two lines.
/// 
/// Given two points per line, determine if the line running indefinetly between
/// respective points ever intersect. If they do not interesect, they are parallell.
/// 
/// Returns the intersection point if the lines intersect, otherwise None.
#[allow(non_snake_case)]
pub(crate) fn do_lines_intersect(start_a: [f32;3], end_a: [f32;3], start_b: [f32;3], end_b: [f32;3]) -> Option<Vector3<f32>>{
    let a1 = end_a[1] - start_a[1];
    let b1 = start_a[0] - end_a[0];
    let c1 = a1 * start_a[0] + b1 * start_a[1];

    let a2 = end_b[1] - start_b[1];
    let b2 = start_b[0] - end_b[0];
    let c2 = a2 * start_b[0] + b2 * start_b[1];

    let determinant = a1 * b2 - a2 * b1;
    if determinant == 0.0 {
        // The lines are parallel
        return None;
    }
    let x = (b2 * c1 - b1 * c2) / determinant;
    let y = (a1 * c2 - a2 * c1) / determinant;
    Some(Vector3::new(x, y, 0.0))
}

/// Computes whether two line segments intersect.
/// 
/// Returns None if the line segments are not intersecting, collinear or parallell and
/// the intersection point if the line segments intersect.
pub(crate) fn do_line_segments_intersect(
        start_a: Vector3<f32>, end_a: Vector3<f32>,
        start_b: Vector3<f32>, end_b: Vector3<f32>) -> Option<Vector3<f32>>{
    let p = start_a;
    let r = end_a - start_a;
    let q = start_b;
    let s = end_b - start_b;

    // define dot product as v * w = vx*wy - vy*wx
    let rs = r.x*s.y - r.y*s.x;
    let qp = q-p;
    let qpr = qp.x*r.y - qp.y*r.x;
    // Line segments are collinear
    if rs == 0.0 && qpr == 0.0 {
        return None;
    }
    // Lines segments are parallell
    if rs == 0.0 && qpr != 0.0 {
        return None;
    }
    let qps = qp.x*s.y - qp.y*s.x;
    let t = qps / rs;
    let u = qpr / rs;
    // Lines segments intersect at the returned point
    if rs != 0.0 &&
            0.0 <= t && t <= 1.0 &&
            0.0 <= u && u <= 1.0 {
        return Some(p + t*r);
    }
    // Lines are not parallell and do not intersect
    return None;
}


/// Computes the closest point on a line running indefinetely
/// between the given line points to a given point.
/// 
/// Returns the closest point on the line.
pub(crate) fn closest_point_on_line(line_start: Vector3<f32>, line_end: Vector3<f32>, p: Vector3<f32>) -> Vector3<f32> {
    // y = ax + b
    let ab = line_end - line_start;
    let t = (p-line_start).dot(ab) / ab.dot(ab);
    let proj = line_start + t * ab;
    return proj;
}

/// Compute the closest point on a line segment to a given point.
/// 
/// Returns the closest point on the line segment.
pub(crate) fn closest_point_on_line_segment(line_start: Vector3<f32>, line_end: Vector3<f32>, p: Vector3<f32>) -> Vector3<f32> {
    let ab = line_end - line_start;
    let t = (p-line_start).dot(ab) / ab.dot(ab);
    let t_clamp = t.clamp(0.0, 1.0);
    let proj = line_start + t_clamp * ab;
    return proj; 
}

/// Computes the point where the circle will collide with the boundary.
/// 
/// The function assumes that the velocity vector of the circle will intersect 
/// with the given line in the next timestep.
/// 
/// Returns the point where the circle will collide with the boundary.
pub(crate) fn compute_collision_point(
        line_start: Vector3<f32>, line_end: Vector3<f32>,
        curr_position: Vector3<f32>, velocity_vec: Vector3<f32>,
            line_collision_point: Vector3<f32>, radius: f32) -> Vector3<f32>{
    let a = line_collision_point;
    let r = radius;
    let c = curr_position;
    let a_c_len = (c).distance(a); // Can be optimized by not taking sqrt (.distance2())
    let p1 = closest_point_on_line(line_start, line_end, c);
    let p1_c = (c).distance(p1); // Can be optimized by not taking sqrt (.distance2())
    let v = f32::sqrt(velocity_vec[0].powi(2) + velocity_vec[1].powi(2));
    let p2_x = a[0] - r * (a_c_len/p1_c)*velocity_vec[0]/v;
    let p2_y = a[1] - r * (a_c_len/p1_c)*velocity_vec[1]/v;

    Vector3::new(p2_x, p2_y, 0.0)
}

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
pub(crate) fn aabb_collision_detection(
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
pub(crate) fn continous_circle_circle_collision_detection(
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



#[cfg(test)]
mod tests {
    mod circle_line_segment_collision_detection {

        use crate::simulation::util::collision_detection::circle_line_segment_collision_detection;

        #[test]
        fn right_angle_movement() {
            let radius = 0.1;
            let line_start = [-1.0, 0.0, 0.0];
            let line_end = [1.0, 0.0, 0.0];
            let curr_pos = [0.0, 1.0, 0.0];
            let mut new_pos = [0.0, 0.100001, 0.0];
        
            let mut res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_eq!(res, None); // No collision as we move just up before we hit the line

            new_pos = [0.0, 0.1, 0.0];
            res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_eq!(res, None); // In this time step there is a "touch" but the movement ends there
        }

        #[test]
        fn on_movement_vector() {
            let radius = 0.1;
            let line_start = [-1.0, 0.0, 0.0];
            let line_end = [1.0, 0.0, 0.0];
            let curr_pos = [0.0, 1.0, 0.0];
            let new_pos = [0.0, -1.0, 0.0];
        
            let res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_ne!(res, None);
            let allowed_diff = 0.0001;
            let expected = 0.45;
            let diff = (res.unwrap() - expected).abs();
            assert!(diff < allowed_diff); 
        }
        #[test]
        fn end_point_less_than_radius_away_from_line() {
            let radius = 0.1;
            let line_start = [-1.0, 0.0, 0.0];
            let line_end = [1.0, 0.0, 0.0];
            let mut curr_pos = [0.0, 1.05, 0.0];
            let mut new_pos = [0.0, 0.05, 0.0];
        
            let mut res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_ne!(res, None);
            let allowed_diff = 0.0001;
            let expected = 0.95;
            let diff = (res.unwrap() - expected).abs();
            assert!(diff < allowed_diff, "Expected {} but got {}. Diff is {}", expected, res.unwrap(), diff); 
        
            curr_pos = [-0.5, 0.5, 0.0];
            new_pos =  [-0.05, 0.05, 0.0];
            res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_ne!(res, None);
            let allowed_diff = 0.0001;
            let expected = 0.9085;
            let diff = (res.unwrap() - expected).abs(); 
            assert!(diff < allowed_diff, "Expected {} but got {}. Diff is {}", expected, res.unwrap(), diff); 
        }

        #[test]
        fn collision_with_line_segment_endpoint() {
            let radius = 0.1;
            let mut line_start = [0.0, 0.0, 0.0];
            let mut line_end = [0.0, -1.0, 0.0];
            let mut curr_pos = [-1.0, 0.01, 0.0];
            let mut new_pos = [1.0, 0.01, 0.0];
        
            let mut res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_ne!(res, None);
            let allowed_diff = 0.0001;
            let expected = 0.45025;
            let diff = (res.unwrap() - expected).abs();
            assert!(diff < allowed_diff, "Expected {} but got {}. Diff is {}", expected, res.unwrap(), diff); 
        
            line_start = [0.0, 1.0, 0.0];
            line_end = [0.0, 0.0, 0.0];
            curr_pos = [-1.0, -0.01, 0.0];
            new_pos = [1.0, -0.01, 0.0];
            res = circle_line_segment_collision_detection(
                line_start.into(), line_end.into(), curr_pos.into(), new_pos.into(), radius);
            assert_ne!(res, None);
            let allowed_diff = 0.0001;
            let expected = 0.45025;
            let diff = (res.unwrap() - expected).abs();
            assert!(diff < allowed_diff, "Expected {} but got {}. Diff is {}", expected, res.unwrap(), diff); 
        }
    }
    mod do_line_segments_intersect {
        use cgmath::Vector3;
        use crate::simulation::util::collision_detection::do_line_segments_intersect;
        
        #[test]
        fn line_segments_are_collinear() {
            let mut line_a_start = [0.0, 0.0, 0.0];
            let mut line_a_end = [1.0, 0.0, 0.0];
            let mut line_b_start = [2.0, 0.0, 0.0];
            let mut line_b_end = [3.0, 0.0, 0.0];
            let mut res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
 
            line_a_start = [0.0, 0.0, 0.0];
            line_a_end = [1.0, 0.0, 0.0];
            line_b_start = [3.0, 0.0, 0.0];
            line_b_end = [2.0, 0.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
 
            line_a_start = [0.0, 1.0, 0.0];
            line_a_end = [1.0, 2.0, 0.0];
            line_b_start = [2.0, 3.0, 0.0];
            line_b_end = [3.0, 4.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
        
        }

        #[test]
        fn line_segments_do_not_intersect() {
            let mut line_a_start = [0.0, 0.0, 0.0];
            let mut line_a_end = [1.0, 0.0, 0.0];
            let mut line_b_start = [-1.0, 1.0, 0.0];
            let mut line_b_end = [-1.0, -1.0, 0.0];
            let mut res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
            
            line_a_start = [0.0, 0.0, 0.0];
            line_a_end = [1.0, 0.0, 0.0];
            line_b_start = [-1.0, -1.0, 0.0];
            line_b_end = [-1.0, 1.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
 
            line_a_start = [0.0, 0.0, 0.0];
            line_a_end = [1.0, 0.0, 0.0];
            line_b_start = [0.0, 2.0, 0.0];
            line_b_end = [0.0, 1.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, None);
        }

        #[test]
        fn line_segments_intersect(){
            let mut line_a_start = [-1.0, 0.0, 0.0];
            let mut line_a_end = [1.0, 0.0, 0.0];
            let mut line_b_start = [0.0, 1.0, 0.0];
            let mut line_b_end = [0.0, -1.0, 0.0];
            let mut res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, Some(Vector3::new(0.0,0.0,0.0)));
            
            line_a_start = [0.0, 1.0, 0.0];
            line_a_end = [0.0, -1.0, 0.0];
            line_b_start = [1.0, 0.0, 0.0];
            line_b_end = [-1.0, 0.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, Some(Vector3::new(0.0,0.0,0.0)));

            line_a_start = [-1.0, -1.0, 0.0];
            line_a_end = [1.0, 1.0, 0.0];
            line_b_start = [-1.0, 1.0, 0.0];
            line_b_end = [1.0, -1.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, Some(Vector3::new(0.0,0.0,0.0)));

            line_a_start = [0.0, 0.0, 0.0];
            line_a_end = [1.0, 0.0, 0.0];
            line_b_start = [0.75, 1.0, 0.0];
            line_b_end = [0.75, -1.0, 0.0];
            res = do_line_segments_intersect(
                line_a_start.into(), line_a_end.into(), 
                         line_b_start.into(), line_b_end.into());
            assert_eq!(res, Some(Vector3::new(0.75,0.0,0.0)));


        }
    }
    mod do_lines_intersect {
        use cgmath::Vector3;

        use crate::simulation::util::collision_detection::do_lines_intersect;

        #[test]
        fn lines_do_not_intersect_line_direction_down(){
            let line_a_start = [0.0, 0.0, 0.0];
            let line_a_end = [1.0, 0.0, 0.0];
            let line_b_start = [-1.0, 1.0, 0.0];
            let line_b_end = [-1.0, -1.0, 0.0];

            let res = do_lines_intersect(
                line_a_start, line_a_end, line_b_start, line_b_end);

            assert_eq!(res, Some(Vector3::new(-1.0, 0.0, 0.0)));
        }

        #[test]
        fn lines_do_not_intersect_line_direction_up(){
            let line_a_start = [0.0, 0.0, 0.0];
            let line_a_end = [1.0, 0.0, 0.0];
            let line_b_start = [-1.0, -1.0, 0.0];
            let line_b_end = [-1.0, 1.0, 0.0];

            let res = do_lines_intersect(
                line_a_start, line_a_end, line_b_start, line_b_end);

            assert_eq!(res, Some(Vector3::new(-1.0, 0.0, 0.0)));
        }

        #[test]
        fn lines_are_parallell(){
            let line_a_start = [0.0, 0.0, 0.0];
            let line_a_end = [1.0, 0.0, 0.0];
            let line_b_start = [0.0, 1.0, 0.0];
            let line_b_end = [1.0, 1.0, 0.0];

            let res = do_lines_intersect(
                line_a_start, line_a_end, line_b_start, line_b_end);

            assert_eq!(res, None);
        }
    }

    mod closest_point_on_line {
        use cgmath::Vector3;

        use crate::simulation::util::collision_detection::closest_point_on_line;

        #[test]
        fn horizontal_bottom() {
            let line_start = Vector3::new(-1.0, -1.0, 0.0);
            let line_end = Vector3::new(1.0, -1.0, 0.0);
            let p = Vector3::new(0.0, 0.0, 0.0);
            let expected_result = Vector3::new(0.0, -1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn horizontal_bottom_negative_x() {
            let line_start = Vector3::new(-1.0, -1.0, 0.0);
            let line_end = Vector3::new(1.0, -1.0, 0.0);
            let p = Vector3::new(-0.5, 0.0, 0.0);
            let expected_result = Vector3::new(-0.5, -1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn horizontal_bottom_positive_x() {
            let line_start = Vector3::new(-1.0, -1.0, 0.0);
            let line_end = Vector3::new(1.0, -1.0, 0.0);
            let p = Vector3::new(0.5, 0.0, 0.0);
            let expected_result = Vector3::new(0.5, -1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn horizontal_top() {
            let line_start = Vector3::new(-1.0, 1.0, 0.0);
            let line_end = Vector3::new(1.0, 1.0, 0.0);
            let p = Vector3::new(0.0, 0.0, 0.0);
            let expected_result = Vector3::new(0.0, 1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn horizontal_top_negative_x() {
            let line_start = Vector3::new(-1.0, 1.0, 0.0);
            let line_end = Vector3::new(1.0, 1.0, 0.0);
            let p = Vector3::new(-0.5, 0.0, 0.0);
            let expected_result = Vector3::new(-0.5, 1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn horizontal_top_positive_x() {
            let line_start = Vector3::new(-1.0, 1.0, 0.0);
            let line_end = Vector3::new(1.0, 1.0, 0.0);
            let p = Vector3::new(0.5, 0.0, 0.0);
            let expected_result = Vector3::new(0.5, 1.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn against_45_degrees_line() {
            let line_start = Vector3::new(-1.0, -1.0, 0.0);
            let line_end = Vector3::new(1.0, 1.0, 0.0);
            let p = Vector3::new(-0.5, 0.5, 0.0);
            let expected_result = Vector3::new(0.0, 0.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn p_on_line() {
            let line_start = Vector3::new(-1.0, -1.0, 0.0);
            let line_end = Vector3::new(1.0, 1.0, 0.0);
            let p = Vector3::new(0.0, 0.0, 0.0);
            let expected_result = Vector3::new(0.0, 0.0, 0.0);
            let res = closest_point_on_line(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
   }

    mod closest_point_on_line_segment {
        use cgmath::Vector3;
        use crate::simulation::util::collision_detection::closest_point_on_line_segment;

        #[test]
        fn closest_point() {
            let line_start = Vector3::new(-1.0, 0.0, 0.0);
            let line_end = Vector3::new(1.0, 0.0, 0.0);
            let p = Vector3::new(0.0, 1.0, 0.0);
            let expected_result = Vector3::new(0.0, 0.0, 0.0);
            let res = closest_point_on_line_segment(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
 
        #[test]
        fn p_not_on_line_segment_but_collinear() {
            let line_start = Vector3::new(0.0, 0.0, 0.0);
            let line_end = Vector3::new(1.0, 0.0, 0.0);
            let p = Vector3::new(2.0, 0.0, 0.0);
            let expected_result = Vector3::new(1.0, 0.0, 0.0);
            let res = closest_point_on_line_segment(line_start, line_end, p);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
    }
    
    mod aabb_collision_detection {
        use cgmath::Vector3;

        use crate::simulation::util::collision_detection::aabb_collision_detection;

        #[test]
        fn x_axis_separated() {
            let radius = 0.1;
            let pos = Vector3::new(-0.2, 0.0, 0.0);
            let new_pos = Vector3::new(-0.1, 0.0, 0.0);
            let pos_other = Vector3::new(0.2, 0.0, 0.0);
            let new_pos_other = Vector3::new(0.3, 0.0, 0.0);
            let expected_result = false;
            let res = aabb_collision_detection(
                pos, new_pos, pos_other, new_pos_other, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn x_axis_collision() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, 0.0, 0.0);
            let new_pos = Vector3::new(0.1, 0.0, 0.0);
            let pos_other = Vector3::new(0.2, 0.0, 0.0);
            let new_pos_other = Vector3::new(0.3, 0.0, 0.0);
            let expected_result = true;
            let res = aabb_collision_detection(
                pos, new_pos, pos_other, new_pos_other, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn y_axis_separated() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, -0.2, 0.0);
            let new_pos = Vector3::new(0.0, -0.1, 0.0);
            let pos_other = Vector3::new(0.0, 0.2, 0.0);
            let new_pos_other = Vector3::new(0.0, 0.3, 0.0);
            let expected_result = false;
            let res = aabb_collision_detection(
                pos, new_pos, pos_other, new_pos_other, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }

        #[test]
        fn y_axis_collision() {
            let radius = 0.1;
            let pos = Vector3::new(0.0, 0.0, 0.0);
            let new_pos = Vector3::new(0.0, 0.1, 0.0);
            let pos_other = Vector3::new(0.0, 0.2, 0.0);
            let new_pos_other = Vector3::new(0.0, 0.3, 0.0);
            let expected_result = true;
            let res = aabb_collision_detection(
                pos, new_pos, pos_other, new_pos_other, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);        
        }
    }

    mod continous_circle_circle_collision_detection {
        use cgmath::Vector3;

        use crate::simulation::util::collision_detection::continous_circle_circle_collision_detection;

        #[test]
        fn no_collision() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(0.2, 0.0, 0.0);
            let circle_1_new_pos = Vector3::new(0.3, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(-0.2, 0.0, 0.0);
            let circle_2_new_pos = Vector3::new(-0.3, 0.0, 0.0);
            let expected_result: Option<f32> = None;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
    
        #[test]
        fn static_dynamic_circles1() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(0.0, 0.0, 0.0);
            let circle_1_new_pos = Vector3::new(0.0, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.25, 0.0, 0.0);
            let circle_2_new_pos = Vector3::new(0.15, 0.0, 0.0);
            let expected_result = 0.5;
            let epsilon: f32 = 0.0001;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_ne!(res, None, "Expected collision but got None");
            let diff = res.unwrap() - expected_result;
            assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn static_dynamic_circles2() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(0.25, 0.0, 0.0);
            let circle_1_new_pos = Vector3::new(0.15, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.0, 0.0, 0.0);
            let circle_2_new_pos = Vector3::new(0.0, 0.0, 0.0);
            let expected_result = 0.5;
            let epsilon: f32 = 0.0001;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_ne!(res, None, "Expected collision but got None");
            let diff = res.unwrap() - expected_result;
            assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn static_static_circles_no_collision() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(-0.15, 0.0, 0.0);
            let circle_1_new_pos = Vector3::new(-0.15, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.15, 0.0, 0.0);
            let circle_2_new_pos = Vector3::new(0.15, 0.0, 0.0);
            let expected_result: Option<f32> = None;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res);
        }
    
        #[test]
        fn dynamic_dynamic_circles_x() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(-0.15, 0.0, 0.0);
            let circle_1_new_pos = Vector3::new(-0.05, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.15, 0.0, 0.0);
            let circle_2_new_pos = Vector3::new(0.05, 0.0, 0.0);
            let expected_result = 0.5;
            let epsilon: f32 = 0.0001;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_ne!(res, None, "Expected collision but got None");
            let diff = res.unwrap() - expected_result;
            assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn dynamic_dynamic_circles_y() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(0.0, -0.15, 0.0);
            let circle_1_new_pos = Vector3::new(0.0, -0.05, 0.0);
            let circle_2_old_pos = Vector3::new(0.0, 0.15, 0.0);
            let circle_2_new_pos = Vector3::new(0.0, 0.05, 0.0);
            let expected_result = 0.5;
            let epsilon: f32 = 0.0001;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_ne!(res, None, "Expected collision but got None");
            let diff = res.unwrap() - expected_result;
            assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn dynamic_dynamic_circles_different_velocities_collide() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(0.0, -0.15, 0.0);
            let circle_1_new_pos = Vector3::new(0.0, -0.05, 0.0);
            let circle_2_old_pos = Vector3::new(0.0, 0.2, 0.0);
            let circle_2_new_pos = Vector3::new(0.0, 0.0, 0.0);
            let expected_result = 0.5;
            let epsilon: f32 = 0.0001;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_ne!(res, None, "Expected collision but got None");
            let diff = res.unwrap() - expected_result;
            assert!(diff.abs() < epsilon, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn dynamic_dynamic_circles_no_collision() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(-0.5, -0.2, 0.0);
            let circle_1_new_pos = Vector3::new(-0.5, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.5, 0.2, 0.0);
            let circle_2_new_pos = Vector3::new(0.5, 0.0, 0.0);
            let expected_result = None;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    
        #[test]
        fn dynamic_dynamic_circles_no_collision_2() {
            let radius = 0.1;
            let circle_1_old_pos = Vector3::new(-0.5, 0.2, 0.0);
            let circle_1_new_pos = Vector3::new(-0.5, 0.0, 0.0);
            let circle_2_old_pos = Vector3::new(0.5, 0.2, 0.0);
            let circle_2_new_pos = Vector3::new(0.5, 0.0, 0.0);
            let expected_result = None;
            let res = continous_circle_circle_collision_detection(
                circle_1_old_pos, circle_1_new_pos, radius, circle_2_old_pos, circle_2_new_pos, radius);
            assert_eq!(res, expected_result, "Expected {:?} but got {:?}", expected_result, res.unwrap());
        }
    }
}