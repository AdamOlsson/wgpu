use std::mem;
use crate::shapes::circle::Circle;

use super::{instance::{self, Instance, InstanceRaw}, vertex::{self, Vertex}, Pass};
use cgmath::prelude::*;
use wgpu::util::DeviceExt;

const NUM_INSTANCES_PER_ROW: u32 = 1;
const INSTANCE_DISPLACEMENT: cgmath::Vector3<f32> = cgmath::Vector3::new(NUM_INSTANCES_PER_ROW as f32 * 0.5, 0.0, NUM_INSTANCES_PER_ROW as f32 * 0.5);
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Locals {
    position: [f32; 4],
    color: [f32; 4],
    normal: [f32; 4],
    lights: [f32; 4],
}

pub struct RenderPass {
    // local_uniform_buffer: wgpu::Buffer,
    // local_bind_group: wgpu::BindGroup,
    render_pipeline: wgpu::RenderPipeline,
    // instances: Vec<Instance>,
    // instance_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,
}

impl RenderPass {
    pub fn new(device: &wgpu::Device,) -> Self {

        let local_size = mem::size_of::<Locals>() as wgpu::BufferAddress;
        // let local_bind_group_layout =
        //     device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        //         label: Some("Locals"),
        //         entries: &[
        //             // Local uniforms
        //             wgpu::BindGroupLayoutEntry {
        //                 binding: 0,
        //                 visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
        //                 ty: wgpu::BindingType::Buffer {
        //                     ty: wgpu::BufferBindingType::Uniform,
        //                     has_dynamic_offset: false,
        //                     min_binding_size: wgpu::BufferSize::new(local_size),
        //                 },
        //                 count: None,
        //             },
        //         ],
        //     });
        
        // Local uniform buffer
        let local_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("[RenderPass] Locals"),
            size: local_size,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // let local_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        //     label: Some("Local Bind Group"),
        //     layout: &local_bind_group_layout,
        //     entries: &[
        //         wgpu::BindGroupEntry {
        //             binding: 0,
        //             resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
        //                 buffer: &local_uniform_buffer,
        //                 offset: 0,
        //                 size: None,
        //             }),
        //         },
        //     ],
        // });

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[],
                push_constant_ranges: &[],
            });

        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Normal Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/shader.wgsl").into()),
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
                    // buffers: &[Vertex::desc(), InstanceRaw::desc()],
                    buffers: &[Vertex::desc()],
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

        // let instances = (0..NUM_INSTANCES_PER_ROW).flat_map(|z| {
        //     (0..NUM_INSTANCES_PER_ROW).map(move |x| {
        //         let position = cgmath::Vector3 { x: x as f32, y: 0.0, z: z as f32 } - INSTANCE_DISPLACEMENT;

        //         let rotation = if position.is_zero() {
        //             // this is needed so an object at (0, 0, 0) won't get scaled to zero
        //             // as Quaternions can affect scale if they're not created correctly
        //             cgmath::Quaternion::from_axis_angle(cgmath::Vector3::unit_z(), cgmath::Deg(0.0))
        //         } else {
        //             cgmath::Quaternion::from_axis_angle(position.normalize(), cgmath::Deg(45.0))
        //         };

        //         instance::Instance {
        //             position, rotation,
        //         }
        //     })
        // }).collect::<Vec<_>>();

        let circle = Circle::new([0.0, 0.0, 0.0], 0.1);
        let vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Circle Vertex Buffer"),
                contents: bytemuck::cast_slice(&circle.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }
        );
        let index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Circle Index Buffer"),
                contents: bytemuck::cast_slice(&circle.indices),
                usage: wgpu::BufferUsages::INDEX,
            }
        );

        let num_indices = circle.num_indices;

        // let instance_data = instances.iter().map(Instance::to_raw).collect::<Vec<_>>();
        // let instance_buffer = device.create_buffer_init(
        //     &wgpu::util::BufferInitDescriptor {
        //         label: Some("Instance Buffer"),
        //         contents: bytemuck::cast_slice(&instance_data),
        //         usage: wgpu::BufferUsages::VERTEX,
        //     }
        // );
         
        RenderPass {
            // local_bind_group,
            // local_uniform_buffer,
            render_pipeline,
            // instances,
            // instance_buffer,
            vertex_buffer,
            index_buffer,
            num_indices
        }
    }
}

impl Pass for RenderPass {
    fn draw(
        &mut self,
        surface: &wgpu::Surface,
        device: &wgpu::Device,
        queue: &wgpu::Queue
    ) -> Result<(), wgpu::SurfaceError> {
        let drawable = surface.get_current_texture()?;
        let image_view_descriptor = wgpu::TextureViewDescriptor::default();
        let image_view = drawable.texture.create_view(&image_view_descriptor);

        let command_encoder_descriptor = wgpu::CommandEncoderDescriptor {
            label: Some("Render Encoder"),
        };

        let mut command_encoder = 
            device.create_command_encoder(&command_encoder_descriptor);

        let color_attachment = wgpu::RenderPassColorAttachment {
            view: &image_view,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.0,
                    g: 0.0,
                    b: 1.0,
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

                // render_pass.set_vertex_buffer(0, self.instance_buffer.slice(..));
                render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
                render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                // render_pass.set_index_buffer(state.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
                render_pass.set_pipeline(&self.render_pipeline);
                
                render_pass.draw_indexed(0..self.num_indices, 0, 0..1);
                // render_pass.draw_indexed(0..self.instances.len() as u32, 0, 0..self.instances.len() as _);
        }

        queue.submit(Some(command_encoder.finish()));

        drawable.present();

        Ok(())
    }
    
}