use crate::renderer_backend::vertex::Vertex;


pub struct Circle {
    pub center: [f32;3],
    pub radius: f32,
    pub indices: Vec<u16>,
    pub num_indices: u32,
    pub vertices: Vec<Vertex>,
}

impl Circle {
    pub fn new(center: [f32;3], radius: f32) -> Self {

        let vertices = Self::compute_vertices(center, radius);
        let indices = Self::compute_indices();

        let num_indices = (359)*3;
        Self { center, radius, indices, num_indices, vertices}
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

    fn compute_vertices(center: [f32;3], radius: f32) -> Vec<Vertex> {
        let x = center[0];
        let y = center[1];
        let mut vertices = Vec::new();
        vertices.push(Vertex { position: [x, y, 0.0], color: [1.0, 0.0, 0.0] });
        for i in 0..360 {
            let angle = i as f32 * std::f32::consts::PI / 180.0;
            vertices.push(Vertex {
                position: [x + radius * angle.cos(), y + radius * angle.sin(), 0.0],
                color: [1.0, 0.0, 0.0],
            });
        }
        return vertices;
    }

    fn compute_indices() -> Vec<u16> {
        let mut indices = Vec::new();
        for i in 1..359 {
            indices.push(i as u16);
            indices.push((i + 1) as u16);
            indices.push(0);
        }
        indices.push(359);
        indices.push(1);
        indices.push(0);
        return indices;
    }
}