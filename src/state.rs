use winit::window::Window;

use crate::{renderer_backend::{graphics_context::GraphicsContext, instance::Instance, render_pass::RenderPass, vertex, Pass}, shapes::circle::Circle};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,

    // TODO: Move to new struct
    circle: Circle,
    velocity: [f32; 3],

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

        let circle = Circle::new([0.0, 0.0, 0.0], 0.1);
        let vertex_buffer = ctx.create_buffer(
            "Circle vertex buffer", bytemuck::cast_slice(&circle.vertices),
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);

        let index_buffer = ctx.create_buffer(
                "Circle index buffer", bytemuck::cast_slice(&circle.indices),
                wgpu::BufferUsages::INDEX);
        
        let instances = vec![
            Instance {position: cgmath::Vector3::new(-0.5, 0.0, 0.0),},
            Instance {position: cgmath::Vector3::new(0.5, 0.0, 0.0),},
        ];
        let instance_buffer = ctx.create_buffer(
            "Circle instance buffer", bytemuck::cast_slice(
                &instances.iter().map(Instance::to_raw).collect::<Vec<_>>()),
                wgpu::BufferUsages::VERTEX);

        let velocity = [0.008, 0.01, 0.0];
        let num_indices = circle.num_indices;
        let num_instances = instances.len() as u32;

        Self { window, ctx, pass, size, circle, velocity, instances, instance_buffer,
                vertex_buffer, index_buffer, num_indices, num_instances}
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn update(&mut self) {
        // if self.circle.center[0] + self.circle.radius + self.velocity[0] >= 1.0 ||
        //         self.circle.center[0] - self.circle.radius + self.velocity[0] <= -1.0 {
        //     self.velocity[0] *= -1.0;
        // }
        // if self.circle.center[1] + self.circle.radius + self.velocity[1] >= 1.0 ||
        //         self.circle.center[1] - self.circle.radius + self.velocity[1] <= -1.0 {
        //     self.velocity[1] *= -1.0;
        // }

        // self.circle.translate(self.velocity);

        // self.ctx.queue.write_buffer(&self.pass.vertex_buffer,
        //      0, bytemuck::cast_slice(&self.circle.vertices));
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue,
            &self.vertex_buffer, &self.index_buffer, &self.instance_buffer,
            self.num_indices, self.num_instances).unwrap();
        return Ok(());
    }
}