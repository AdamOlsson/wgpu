use rand::Rng;
use winit::window::Window;

use crate::{renderer_backend::{graphics_context::GraphicsContext, instance::Instance, render_pass::RenderPass, vertex, Pass}, shapes::circle::{self, Circle}};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,

    // TODO: Move to new struct
    pub circles: Vec<Circle>,

    instances: Vec<Instance>,
    instance_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    num_indices: u32,
    num_instances: u32,
}

impl <'a> State <'a> {
    pub async fn new(window: &'a Window) -> Self {
        let size = window.inner_size();

        let mut ctx = GraphicsContext::new(&window).await;

        let pass = RenderPass::new(&ctx.device);

        let base_circle = Circle::new([0.0, 0.0, 0.0], 0.1);

        let mut circles = vec![
            Circle::new([0.5, 0.0, 0.0], 0.1),
            Circle::new([-0.5, 0.0, 0.0], 0.1),
            Circle::new([-0.5, 0.5, 0.0], 0.1),
            Circle::new([0.5, 0.5, 0.0], 0.1),
        ];

        let mut rng = rand::thread_rng();
        let max = 0.01;
        let min = -0.01;
        for c in circles.iter_mut() {
            c.velocity = [rng.gen_range(min..=max), rng.gen_range(min..=max), 0.0];
        }
    
        let vertex_buffer = ctx.create_buffer(
            "Circle vertex buffer", bytemuck::cast_slice(&base_circle.vertices),
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);

        let index_buffer = ctx.create_buffer(
                "Circle index buffer", bytemuck::cast_slice(&base_circle.indices),
                wgpu::BufferUsages::INDEX);
        
        let instances = circles.iter().map(
            |circle| Instance {
                position: cgmath::Vector3::new(circle.position[0], circle.position[1], circle.position[2]),
            }).collect::<Vec<_>>();


        let instance_buffer = ctx.create_buffer(
            "Circle instance buffer", bytemuck::cast_slice(
                &instances.iter().map(Instance::to_raw).collect::<Vec<_>>()),
                wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);

        let num_indices = base_circle.num_indices;
        let num_instances = instances.len() as u32;

        Self { window, ctx, pass, size, instances, instance_buffer,
                vertex_buffer, index_buffer, num_indices, num_instances,
                 circles
                }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn update(&mut self) {
        for circle in self.circles.iter_mut() {
            circle.update();
        }

        let positions = self.circles.iter().map(|c| c.position).collect::<Vec<_>>();
        
        self.ctx.queue.write_buffer(&self.instance_buffer, 
             0, bytemuck::cast_slice(
                &positions));
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue,
            &self.vertex_buffer, &self.index_buffer, &self.instance_buffer,
            self.num_indices, self.num_instances).unwrap();
        return Ok(());
    }
}