use wgpu::{core::device, BindGroup, BindGroupDescriptor, BindGroupEntry, BindGroupLayout, BindGroupLayoutDescriptor, BindGroupLayoutEntry, BindingType, BufferAddress, CommandEncoderDescriptor, Device, ImageCopyTexture, ImageDataLayout, Origin3d, PipelineLayoutDescriptor, Queue, RenderPipeline, RenderPipelineDescriptor, Sampler, SamplerDescriptor, ShaderModuleDescriptor, ShaderStages, Surface, Texture, TextureDescriptor, TextureUsages, TextureView, TextureViewDescriptor, VertexAttribute, VertexBufferLayout, VertexState, VertexStepMode};

use crate::renderer_backend::render_pass;


pub struct Bloom {
    render_pipeline: RenderPipeline,
    input_texture: Texture,
    sampler_bind_group: BindGroup,
    texture_bind_group: BindGroup

}

impl Bloom {

    pub fn new(device: &wgpu::Device) -> Self {
        let (sampler, sampler_bind_group, sampler_bind_group_layout) = Self::init_sampler(device);
        // TODO: Write surface texture to this texture
        let (texture, texture_bind_group, texture_bind_group_layout) = Self::init_texture(device);

        let bloom_pipeline_layout = device.create_pipeline_layout(
            &PipelineLayoutDescriptor { 
                label: Some("Bloom Pipeline Layout"),
                bind_group_layouts: &[
                    &sampler_bind_group_layout,
                    &texture_bind_group_layout
                ],
                push_constant_ranges: &[]
            }); 

        let render_shader = device.create_shader_module(
            ShaderModuleDescriptor {
                label: Some("Bloom Render Shader"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shaders/bloom.wgsl").into())
            }); 
        
        let render_targets = [Some(wgpu::ColorTargetState {
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            blend: Some(wgpu::BlendState::REPLACE),
            write_mask: wgpu::ColorWrites::ALL,
        })];

        let render_pipeline = device.create_render_pipeline(
            &RenderPipelineDescriptor {
                label: Some("Bloom Render Pipeline"),
                layout: Some(&bloom_pipeline_layout),
                vertex: VertexState {
                    module: &render_shader,
                    entry_point: "vs_main",
                    buffers: &[ Self::create_vertex_buffer_layout() ]
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
                    module: &render_shader,
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
            });

        Self { render_pipeline, sampler_bind_group, texture_bind_group, input_texture: texture }
    }

    pub fn render(
        &mut self, surface: &Surface, device: &Device, queue: &Queue
    ) -> Result<(), wgpu::SurfaceError> {
        let drawable = surface.get_current_texture()?;
        let image_view = drawable.texture.create_view(&TextureViewDescriptor::default());

        let mut command_encoder = device.create_command_encoder(
            &CommandEncoderDescriptor { label: Some("Bloom Command Encoder") });
        
        {
            let mut render_pass = command_encoder.begin_render_pass(
                &wgpu::RenderPassDescriptor {
                    label: Some("Render Pass"),
                    color_attachments: &[
                        Some(
                            wgpu::RenderPassColorAttachment {
                                view: &image_view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load, 
                                    store: wgpu::StoreOp::Store,
                                },
                            }
                        )],
                    depth_stencil_attachment: None,
                    occlusion_query_set: None,
                    timestamp_writes: None,
                });
        }

        Ok(())

    }

    fn init_texture(device: &wgpu::Device) -> (Texture, BindGroup, BindGroupLayout) { 
        let bloom_texture = device.create_texture(
            &TextureDescriptor {
                label: Some("Bloom Texture"),
                size: wgpu::Extent3d {
                    width: 800,
                    height: 800,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Bgra8UnormSrgb,
                usage:  TextureUsages::COPY_DST |
                        TextureUsages::STORAGE_BINDING |
                        TextureUsages::TEXTURE_BINDING,
                view_formats: &[]
            });
        
        let bloom_texture_view = bloom_texture.create_view(&TextureViewDescriptor::default());
        
        let bloom_texture_bind_group_layout = device.create_bind_group_layout(
            &BindGroupLayoutDescriptor {
                 label: Some("Bloom Texture Bind Group Layout"),
                 entries: &[
                    // Input Texture
                    BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::all(),
                        ty: BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false
                        }, 
                        count: None
                    },
                    // Output Texture
                    BindGroupLayoutEntry {
                        binding: 1,
                        visibility: ShaderStages::all(),
                        ty: BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false
                        }, 
                        count: None
                    }
                 ] });

        let bloom_texture_bind_group = device.create_bind_group(
            &BindGroupDescriptor {
                    label: Some("Bloom Constants Bind Group"),
                    layout: &bloom_texture_bind_group_layout,
                    entries: &[
                        BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::TextureView(&bloom_texture_view),
                        }
                    ]
                });

        return (bloom_texture, bloom_texture_bind_group, bloom_texture_bind_group_layout);
    }

    fn init_sampler(device: &wgpu::Device) -> (Sampler, BindGroup, BindGroupLayout) {
        let bloom_sampler = device.create_sampler(&SamplerDescriptor {
            label: Some("Bloom Sampler"), 
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge, 
            mag_filter: wgpu::FilterMode::Linear, min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
 
        let bloom_sampler_bind_group_layout = device.create_bind_group_layout(
            &BindGroupLayoutDescriptor {
                    label: Some("Bloom Sampler Bind Group Layout"),
                    entries: &[
                        BindGroupLayoutEntry {
                            binding: 0,
                            visibility: ShaderStages::all(),
                            ty: BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                            count: None }
                    ]
                }
        );
        let bloom_sampler_bind_group = device.create_bind_group(
            &BindGroupDescriptor {
                label: Some("Bloom Sampler Bind Group"),
                layout: &bloom_sampler_bind_group_layout,
                entries: &[
                    BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::Sampler(&bloom_sampler) 
                    }
                ]
            });
        return (bloom_sampler, bloom_sampler_bind_group, bloom_sampler_bind_group_layout);
    }
    
    fn create_vertex_buffer_layout() -> VertexBufferLayout<'static> {
        VertexBufferLayout {
            array_stride: std::mem::size_of::<[f32;3]>() as BufferAddress,
            step_mode: VertexStepMode::Vertex,
            attributes: &[
                VertexAttribute {
                    offset: 0, 
                    shader_location: 0, 
                    format: wgpu::VertexFormat::Float32x3,
                    }
            ] }
    }
}