use winit::window::Window;

use crate::{renderer_backend::{graphics_context::GraphicsContext, render_pass::RenderPass, Pass}, shapes::circle::Circle};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,

    // TODO: Move to new struct
    circle: Circle,
    velocity: [f32; 3],
}

impl <'a> State <'a> {
    pub async fn new(window: &'a Window) -> Self {
        let size = window.inner_size();

        let ctx = GraphicsContext::new(&window).await;

        let pass = RenderPass::new(&ctx.device);

        let circle = Circle::new([0.0, 0.0, 0.0], 0.1);
        let velocity = [0.008, 0.01, 0.0];

        Self { window, ctx, pass, size, circle, velocity}
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn update(&mut self) {
        if self.circle.center[0] + self.circle.radius + self.velocity[0] >= 1.0 ||
                self.circle.center[0] - self.circle.radius + self.velocity[0] <= -1.0 {
            self.velocity[0] *= -1.0;
        }
        if self.circle.center[1] + self.circle.radius + self.velocity[1] >= 1.0 ||
                self.circle.center[1] - self.circle.radius + self.velocity[1] <= -1.0 {
            self.velocity[1] *= -1.0;
        }

        self.circle.translate(self.velocity);

        self.ctx.queue.write_buffer(&self.pass.vertex_buffer,
             0, bytemuck::cast_slice(&self.circle.vertices));
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue).unwrap();
        return Ok(());
    }
}