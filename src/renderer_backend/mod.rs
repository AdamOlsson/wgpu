use wgpu::{Device, Queue, Surface};

pub mod vertex;
pub mod render_pass;
pub mod instance;
pub mod graphics_context;

pub trait Pass {
    fn draw(
        &mut self,
        surface: &Surface,
        device: &Device,
        queue: &Queue,
    ) -> Result<(), wgpu::SurfaceError>;
}