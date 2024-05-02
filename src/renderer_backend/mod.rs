pub mod vertex;
pub mod render_pass;
pub mod instance;
pub mod graphics_context;

use wgpu::{Device, Queue, Surface};
pub trait Pass {
    fn draw(
        &mut self,
        surface: &Surface,
        device: &Device,
        queue: &Queue,
        vertex_buffer: &wgpu::Buffer,
        index_buffer: &wgpu::Buffer,
        instance_buffer: &wgpu::Buffer,
        num_indices: u32,
        num_instances: u32,
    ) -> Result<(), wgpu::SurfaceError>;
}