use core::num;

use crate::renderer_backend::vertex::Vertex;


pub struct Triangle {
    pub center: [f32;3],
    pub base: f32,
    pub height: f32,
    pub indices: Vec<u16>,
    pub num_indices: u32,
    pub vertices: Vec<Vertex>,
}

impl Triangle {
    pub fn new(center: [f32;3], base: f32, height: f32) -> Self {
        let indices = vec![0, 1, 2];
        let num_indices = 3;
        let vertices = Self::get_vertices(center, base, height);
        Self { center, base, height, indices, num_indices, vertices}
    }

    pub fn translate(&mut self, translation: [f32;3]) {
        self.center[0] += translation[0];
        self.center[1] += translation[1];
        self.center[2] += translation[2];
        for vertex in self.vertices.iter_mut() {
            vertex.position[0] += translation[0];
            vertex.position[1] += translation[1];
            vertex.position[2] += translation[2];
        }
    }

    fn get_vertices(center: [f32;3], base: f32, height: f32) -> Vec<Vertex> {
        let x = center[0];
        let y = center[1];
        let base = base;
        let height = height;
        let mut vertices = Vec::new();
        vertices.push(Vertex { position: [x, y + height / 2.0, 0.0], color: [1.0, 0.0, 0.0] });
        vertices.push(Vertex { position: [x - base / 2.0, y - height / 2.0, 0.0], color: [1.0, 0.0, 0.0] });
        vertices.push(Vertex { position: [x + base / 2.0, y - height / 2.0, 0.0], color: [1.0, 0.0, 0.0] });
        return vertices;
    }
}