use winit::window::Window;

use crate::{collision_simulation::CollisionSimulation, renderer_backend::{graphics_context::GraphicsContext, instance::Instance, render_pass::RenderPass, vertex, Pass}, shapes::circle::{self, Circle}};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,

    collision_simulation: CollisionSimulation,
    instance_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
}

impl <'a> State <'a> {
    pub async fn new(window: &'a Window) -> Self {
        let size = window.inner_size();

        let mut ctx = GraphicsContext::new(&window).await;

        let pass = RenderPass::new(&ctx.device);
        
        let collision_simulation = CollisionSimulation::new();
    
        let vertex_buffer = ctx.create_buffer(
            "Circle vertex buffer", bytemuck::cast_slice(&collision_simulation.vertices),
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);

        let index_buffer = ctx.create_buffer(
                "Circle index buffer", bytemuck::cast_slice(&collision_simulation.indices),
                wgpu::BufferUsages::INDEX);
        
        let instances = (0..collision_simulation.num_instances as usize).map(
            |i| Instance {
                position: collision_simulation.positions[i].into(),
                color: collision_simulation.colors[i].into(),
            }).collect::<Vec<_>>();

        let instance_buffer = ctx.create_buffer(
            "Circle instance buffer", bytemuck::cast_slice(
                &instances.iter().map(Instance::to_raw).collect::<Vec<_>>()),
                wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);


        Self { window, ctx, pass, size, instance_buffer,
                vertex_buffer, index_buffer, 
                collision_simulation
                }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn update(&mut self) {
        self.collision_simulation.update();

        let mut instance_data: Vec<[[f32;3];2]> = Vec::new();
        for i in 0..self.collision_simulation.num_instances as usize {
            instance_data.push([self.collision_simulation.positions[i], self.collision_simulation.colors[i]]);
        }
        
        // To prevent writing the static colors every run, we probably can use a global buffer and write 
        // the colors to it once (maybe and then copy it to the instance buffer every frame.)
        self.ctx.queue.write_buffer(&self.instance_buffer, 
             0, bytemuck::cast_slice(
                &instance_data));
        
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue,
            &self.vertex_buffer, &self.index_buffer, &self.instance_buffer,
            self.collision_simulation.num_indices,
            self.collision_simulation.num_instances
        ).unwrap();
        return Ok(());
    }
}