use winit::window::Window;

use crate::{renderer_backend::{graphics_context::GraphicsContext, instance::Instance, render_pass::RenderPass, Pass}, simulation::{particle_fire_simulation::ParticleFireSimulation, verlet_collision_simulation::VerletCollisionSimulation}};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,

    //simulation: GravitySimulation,
    //simulation: CollisionSimulation,
    //simulation: VerletIntegration,
    //simulation: VerletCollisionSimulation,
    simulation: ParticleFireSimulation,

    instance_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
}

impl <'a> State <'a> {
    pub async fn new(window: &'a Window) -> Self {
        let size = window.inner_size();

        let mut ctx = GraphicsContext::new(&window).await;

        let pass = RenderPass::new(&ctx.device);
        
        //let simulation = CollisionSimulation::new();
        //let simulation = GravitySimulation::new();
        //let simulation = VerletIntegration::new();
        //let simulation = VerletCollisionSimulation::new();
        let simulation = ParticleFireSimulation::new();
    
        let vertex_buffer = ctx.create_buffer(
            "Circle vertex buffer", bytemuck::cast_slice(&simulation.vertices),
            wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);

        let index_buffer = ctx.create_buffer(
                "Circle index buffer", bytemuck::cast_slice(&simulation.indices),
                wgpu::BufferUsages::INDEX);
        
        let positions = simulation.get_positions();
        let radii = simulation.get_radii();
        let instances = (0..simulation.get_target_num_instances() as usize).map(
            |i| Instance {
                position: positions[i].into(),
                color: simulation.colors[i].into(),
                radius: radii[i],
            }).collect::<Vec<_>>();

        let instance_buffer = ctx.create_buffer(
            "Circle instance buffer", bytemuck::cast_slice(
                &instances.iter().map(Instance::to_raw).collect::<Vec<_>>()),
                wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST);


        Self { window, ctx, pass, size, instance_buffer,
                vertex_buffer, index_buffer, 
                simulation
                }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn update(&mut self) {
        self.simulation.update();

        let positions = self.simulation.get_positions();
        let radii = self.simulation.get_radii();
        let instances = (0..self.simulation.get_target_num_instances() as usize).map(
            |i| Instance {
                position: positions[i].into(),
                color: self.simulation.colors[i].into(),
                radius: radii[i],
            }.to_raw()).collect::<Vec<_>>();

        // To prevent writing the static colors every run, we probably can use a global buffer and write 
        // the colors to it once (maybe and then copy it to the instance buffer every frame.)
        self.ctx.queue.write_buffer(&self.instance_buffer, 
             0, bytemuck::cast_slice(&instances));
        
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue,
            &self.vertex_buffer, &self.index_buffer, &self.instance_buffer,
            self.simulation.num_indices,
            self.simulation.get_num_active_instances(),
        ).unwrap();
        return Ok(());
    }
}