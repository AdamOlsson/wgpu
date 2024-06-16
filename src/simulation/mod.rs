use cgmath::{InnerSpace, Vector3};

pub mod collision_simulation;
pub mod spatial_subdivision;
pub mod gravity_simulation;

mod util;
pub mod verlet_collision_simulation;
pub mod particle_fire_simulation;
pub mod solvers;

#[allow(dead_code)]
pub fn circle_constraint(
    pos: &mut Vector3<f32>, radius: &f32
)  {
    let constraint_center = Vector3::new(0.0,0.0,0.0);
    let constraint_radius = &0.95;

    let diff = *pos - constraint_center;
    let dist = diff.magnitude();
    if dist > (constraint_radius - radius) {
        let correction_direction = diff / dist;
        *pos = constraint_center + correction_direction*(constraint_radius - radius);
    }
}

#[allow(dead_code)]
pub fn box_constraint(
    pos: &mut Vector3<f32>, radius: &f32
) { 
    let constraint_top_left = Vector3::new(-1.0, 1.0, 0.0);
    let constraint_bottom_right = Vector3::new(1.0, -1.0, 0.0);
    // Left side
    if pos.x - radius < constraint_top_left.x {
        let diff = pos.x - radius  - constraint_top_left.x;
        pos.x -= diff*2.0;
    }
    // Right side
    if pos.x + radius > constraint_bottom_right.x {
        let diff = pos.x + radius - constraint_bottom_right.x; 
        pos.x -= diff*2.0;
    }
    // Bottom side
    if pos.y - radius < constraint_bottom_right.y {
        let diff = pos.y - radius - constraint_bottom_right.y;
        pos.y -= diff*2.0;
    }
    // Top side
    if pos.y + radius > constraint_top_left.y {
        let diff = pos.y + radius - constraint_top_left.y;
        pos.y -= diff*2.0;
    }
}