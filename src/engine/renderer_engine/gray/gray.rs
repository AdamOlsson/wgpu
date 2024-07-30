
use util::{BufferInitDescriptor, DeviceExt};
use::wgpu::*;

pub struct Gray {
    render_pipeline: RenderPipeline,
    vertex_buffer: Buffer,
    bind_group: BindGroup,
    pub texture: Texture
}

impl Gray {
    pub fn new(device: &Device, size: &winit::dpi::PhysicalSize<u32>) -> Self {
        let texture = Self::create_texture(device, size);
        let sampler = Self::create_sampler(device);
        let (vertex_buffer, vertex_buffer_layout) = Self::create_vertex_buffer(device);
        let (bind_group, bind_group_layout) = Self::create_bind_group(device, &sampler, &texture);
        
        let render_pipeline = Self::create_pipeline(device, &[&bind_group_layout], vertex_buffer_layout);

        Self { render_pipeline, texture, vertex_buffer, bind_group }
    }

    pub fn render(&mut self, target_texture: &Texture, device: &Device, queue: &Queue) -> Result<(), wgpu::SurfaceError> {

        let mut command_encoder = device.create_command_encoder(
            &CommandEncoderDescriptor { label: Some("Gray Command Encoder") });
        {
            let target_texture_view = target_texture.create_view(&TextureViewDescriptor::default());
            let mut render_pass = command_encoder.begin_render_pass(
                &wgpu::RenderPassDescriptor {
                    label: Some("Render Pass"),
                    color_attachments: &[
                        Some(
                            wgpu::RenderPassColorAttachment {
                                view: &target_texture_view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(
                                        wgpu::Color {
                                            r: 0.0,
                                            g: 0.0,
                                            b: 0.0,
                                            a: 1.0,
                                    }), 
                                    store: wgpu::StoreOp::Store,
                                },
                            }
                        )],
                    depth_stencil_attachment: None,
                    occlusion_query_set: None,
                    timestamp_writes: None,
                });
            
            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_bind_group(0, &self.bind_group, &[]);
    
            render_pass.draw(0..6, 0..1);
        }

        queue.submit(Some(command_encoder.finish()));

        Ok(())
    }

    fn create_texture(device: &wgpu::Device, size: &winit::dpi::PhysicalSize<u32>) -> Texture { 
        device.create_texture(
            &TextureDescriptor {
                label: Some("Gray Texture"),
                size: wgpu::Extent3d {
                    width: size.width,
                    height: size.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Bgra8UnormSrgb,
                usage:  TextureUsages::COPY_DST |
                        TextureUsages::RENDER_ATTACHMENT |
                        TextureUsages::TEXTURE_BINDING,
                view_formats: &[]
            })
    }

    fn create_sampler(device: &wgpu::Device) -> Sampler {
        device.create_sampler(&SamplerDescriptor {
            label: Some("Gray Sampler"), 
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge, 
            mag_filter: wgpu::FilterMode::Linear, min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        })
    }

    fn create_vertex_buffer(device: &wgpu::Device) -> (Buffer, VertexBufferLayout) {
        let layout = VertexBufferLayout {
            array_stride: std::mem::size_of::<[f32; 6]>() as BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                // Positions
                VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                // Texture coord
                VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ]
        };

        let vertices: [[f32;3];12] = [
            // Positions     // Tex Coord
            [-1., -1.,  0.], [ 0., 1., 0.], // Bottom left
            [ 1., -1.,  0.], [ 1., 1., 0.], // Bottom right
            [-1.,  1.,  0.], [ 0., 0., 0.], // Top left
            
            [ 1.,  1.,  0.], [ 1., 0., 0.], // Top right
            [-1.,  1.,  0.], [ 0., 0., 0.], // Top left
            [ 1., -1.,  0.], [ 1., 1., 0.], // Bottom right
        ];
        let buffer = device.create_buffer_init(
            &BufferInitDescriptor { 
                label: Some("Gray Vertice Buffer"),
                contents: &bytemuck::cast_slice(&vertices),
                usage: BufferUsages::VERTEX });
        (buffer, layout)
    }

    fn create_bind_group(
        device: &wgpu::Device, sampler: &Sampler, texture: &Texture
    ) -> (BindGroup, BindGroupLayout) {
        let layout = device.create_bind_group_layout(
            &BindGroupLayoutDescriptor {
                label: Some("Gray Bind Group Layout"),
                entries: &[
                    BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                    BindGroupLayoutEntry {
                        binding: 1,
                        visibility: ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false },
                        count: None }
                ] }
        );
        let bind_group = device.create_bind_group(
            &BindGroupDescriptor {
                label: Some("Gray Bind Group"),
                layout: &layout,
                entries: &[
                    BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::Sampler(sampler) 
                    },
                    BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(
                            &texture.create_view(&TextureViewDescriptor::default())) 
                    }
                ] }
        );
        (bind_group, layout)
    }

    fn create_pipeline(
        device: &wgpu::Device, bind_group_layouts: &[&BindGroupLayout],
        vertex_buffer_layout: VertexBufferLayout
    ) -> RenderPipeline {
        let render_shader = device.create_shader_module(
            ShaderModuleDescriptor {
                label: Some("Gray Render Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shaders/gray.wgsl").into())
            }); 
        
        let render_targets = [Some(wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            blend: Some(wgpu::BlendState::REPLACE),
            write_mask: wgpu::ColorWrites::ALL,
        })];

        let layout = device.create_pipeline_layout(
            &PipelineLayoutDescriptor {
                label: Some("Gray Pipeline Layout"),
                bind_group_layouts: bind_group_layouts,
                push_constant_ranges: &[] }
        );

        let pipeline = device.create_render_pipeline(
            &RenderPipelineDescriptor {
                label: Some("Gray Render Pipeline"),
                layout: Some(&layout),
                vertex: VertexState {
                    module: &render_shader,
                    entry_point: "vs_main",
                    buffers: &[ vertex_buffer_layout ]
                },
                primitive: PrimitiveState {
                    topology: wgpu::PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: Some(wgpu::Face::Back),
                    polygon_mode: wgpu::PolygonMode::Fill,
                    unclipped_depth: false,
                    conservative: false,
                }, 
                depth_stencil: None,
                multisample: MultisampleState {
                    count: 1,
                    mask: !0,
                    alpha_to_coverage_enabled: false
                }, 
                fragment: Some(FragmentState {
                    module: &render_shader,
                    entry_point: "fs_main",
                    targets: &render_targets
                }),
                multiview: None
            });

        pipeline
    }
}
