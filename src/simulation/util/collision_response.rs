use cgmath::{InnerSpace, Vector3};


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
pub(crate) fn circle_circle_collision_response(
        ttc: f32, 
        va: &mut Vector3<f32>, vb: &mut Vector3<f32>,
        pa: &Vector3<f32>, pb: &Vector3<f32>,
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