
use wgpu::{ util::{BufferInitDescriptor, DeviceExt}, BindGroup, BindGroupLayout, BindGroupLayoutEntry, Buffer, BufferBindingType, BufferUsages, ShaderStages, Texture, TextureViewDescriptor};
use super::{instance::InstanceRaw, vertex::Vertex, Pass};

pub struct RenderPass {
    //buffer: wgpu::Buffer,
    buffer_bind_group: wgpu::BindGroup,
    render_pipeline: wgpu::RenderPipeline,
}

impl RenderPass {
    pub fn new(device: &wgpu::Device, window_size: &winit::dpi::PhysicalSize<u32>) -> Self {
        let size = [window_size.width as f32, window_size.height as f32];
        let (_buffer, buffer_bind_group, buffer_bind_group_layout) = Self::create_uniform_buffer(device, &size);

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[
                    &buffer_bind_group_layout
                ],
                push_constant_ranges: &[],
            });

        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Normal Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/shader.wgsl").into()),
        });
        
        let render_targets = [Some(wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            blend: Some(wgpu::BlendState::REPLACE),
            write_mask: wgpu::ColorWrites::ALL,
        })];


        let render_pipeline = device.create_render_pipeline(
            &wgpu::RenderPipelineDescriptor {
                label: Some("Render Pipeline"),
                layout: Some(&render_pipeline_layout),
    
                vertex: wgpu::VertexState {
                    module: &shader_module,
                    entry_point: "vs_main",
                    buffers: &[Vertex::desc(), InstanceRaw::desc()],
                },
    
                primitive: wgpu::PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back),
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                },
    
                fragment: Some(wgpu::FragmentState {
                    module: &shader_module,
                    entry_point: "fs_main",
                    targets: &render_targets,
                }),
    
                depth_stencil: None,
                multisample: wgpu::MultisampleState {
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                multiview: None,
            }
        );

        RenderPass {
            render_pipeline,
            //buffer,
            buffer_bind_group
        }
    }

    fn create_uniform_buffer(device: &wgpu::Device, data: &[f32]) -> (Buffer, BindGroup, BindGroupLayout) {
        let uniform_buffer = device.create_buffer_init(
            &BufferInitDescriptor {
                label: Some("Global render information"),
                contents: bytemuck::cast_slice(data),
                usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST
        });

        let uniform_buffer_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor { 
                label: Some("Global render buffer layout"),
                entries: &[
                    BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    }
                ]
            }
        );

        let uniform_buffer_bind_group = device.create_bind_group(
            &wgpu::BindGroupDescriptor {
                label: Some("Global render information bind group"),
                layout: &uniform_buffer_group_layout, 
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: uniform_buffer.as_entire_binding(),
                    },
                ]
            }
        );

        (uniform_buffer, uniform_buffer_bind_group, uniform_buffer_group_layout)
    }
}

impl Pass for RenderPass {
    fn draw(
        &mut self,
        target_texture: &Texture,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        vertex_buffer: &wgpu::Buffer,
        index_buffer: &wgpu::Buffer,
        instance_buffer: &wgpu::Buffer,
        num_indices: u32,
        num_instances: u32,

    ) -> Result<(), wgpu::SurfaceError> {
        
        let command_encoder_descriptor = wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        };

        let mut command_encoder = 
            device.create_command_encoder(&command_encoder_descriptor);

        let color_attachment = wgpu::RenderPassColorAttachment {
            view: &target_texture.create_view(&TextureViewDescriptor::default()),
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.0,
                    g: 0.0,
                    b: 0.0,
                    a: 1.0,
                }),
                store: wgpu::StoreOp::Store,
            },
        };

        {
            let mut render_pass = command_encoder.begin_render_pass(
                &wgpu::RenderPassDescriptor {
                    label: Some("Render Pass"),
                    color_attachments: &[Some(color_attachment)],
                    depth_stencil_attachment: None,
                    occlusion_query_set: None,
                    timestamp_writes: None,
                });

                render_pass.set_bind_group(0, &self.buffer_bind_group, &[]);
                render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                render_pass.set_vertex_buffer(1, instance_buffer.slice(..));
                
                render_pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                render_pass.set_pipeline(&self.render_pipeline);
                
                render_pass.draw_indexed(0..num_indices, 0, 0..num_instances);
        }

        queue.submit(Some(command_encoder.finish()));

        Ok(())
    }
    
}
