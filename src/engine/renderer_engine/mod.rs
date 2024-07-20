pub mod vertex;
//pub mod bloom;
pub mod gray;
pub mod graphics_context;
pub mod render_pass;
pub mod instance;
pub mod shapes;

use wgpu::{Device, Queue, Texture};
pub trait Pass {
    fn draw(
        &mut self,
        target_texture: &Texture,
        device: &Device,
        queue: &Queue,
        vertex_buffer: &wgpu::Buffer,
        index_buffer: &wgpu::Buffer,
        instance_buffer: &wgpu::Buffer,
        num_indices: u32,
        num_instances: u32,
    ) -> Result<(), wgpu::SurfaceError>;
}