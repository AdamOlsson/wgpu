use winit::window::Window;

use crate::renderer_backend::{graphics_context::GraphicsContext, render_pass::RenderPass, Pass};


pub struct State<'a> {
    pub window: &'a Window,
    ctx: GraphicsContext<'a>,
    pass: RenderPass,
    size: winit::dpi::PhysicalSize<u32>,
    clear_color: wgpu::Color,
}

impl <'a> State <'a> {
    pub async fn new(window: &'a Window) -> Self {
        let size = window.inner_size();

        let ctx = GraphicsContext::new(&window).await;
        let clear_color = wgpu::Color::BLACK;

        let pass = RenderPass::new(&ctx.device);
        Self { window, ctx, pass, size, clear_color }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.size = new_size;
        self.ctx.config.width = new_size.width;
        self.ctx.config.height = new_size.height;
        self.ctx.surface.configure(&self.ctx.device, &self.ctx.config);
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.pass.draw(&self.ctx.surface, &self.ctx.device, &self.ctx.queue).unwrap();
        return Ok(());
    }
}