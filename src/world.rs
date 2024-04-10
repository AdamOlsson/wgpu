use crate::renderer_backend::vertex::Vertex;
// const COLOR1: [f32; 3] = [0.20, 0.19, 0.17]; // Underground
// const COLOR2: [f32; 3] = [0.32, 0.20, 0.18]; // Surface
// const COLOR3: [f32; 3] = [0.46, 0.36, 0.26];
// const COLOR4: [f32; 3] = [0.61, 0.59, 0.42];
// const COLOR5: [f32; 3] = [0.66, 0.71, 0.60];
// const COLOR6: [f32; 3] = [0.75, 0.76, 0.76];
// const COLOR7: [f32; 3] = [0.53, 0.55, 0.65]; // Sky
// const COLOR8: [f32; 3] = [0.42, 0.35, 0.57];

pub struct World {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub num_indices: u32,
}

impl World {
    pub fn new(color: [f32;3]) -> Self {
        let world_width = 10;
        let world_height = 10;
        let vertices = Self::build_vertex_grid(world_width, world_height, color);
        let indices = Self::build_index_grid(world_width, world_height);

        // Self::generate_surface(&mut vertices);


        let num_indices = indices.len() as u32;
        return World { vertices, indices, num_indices };
    }

    fn build_vertex_grid(width: u32, height: u32, color: [f32;3]) -> Vec<Vertex> {
        let vertex_delta_x = 2.0 / width as f32;
        let vertex_delta_y = 2.0 / height as f32;
        println!("Vertex delta x: {}, Vertex delta y: {}", vertex_delta_x, vertex_delta_y);
        let mut vertices = vec![];
        for i in 0..(width+1) {
            for j in 0..(height+1) {
                let x = (i as f32 * vertex_delta_x) - 1.0;
                let y = (j as f32 * vertex_delta_y) - 1.0;
                vertices.push(Vertex { position: [x, y, 0.0], color: color});
            }
        }
        return vertices;
    }

    fn build_index_grid(width: u32, height: u32) -> Vec<u16> {
        let num_vertices_x = width + 1;
        let num_vertices_y = height + 1;
        let num_vertices = num_vertices_x * num_vertices_y;
        
        // Each iteration adds 2 triangles forming a square, skip last row and last column
        let mut indices = vec![];
        for i in 0..(num_vertices - num_vertices_x) {
            // Check if the vertex is at the right edge
            if (i + 1) % num_vertices_x == 0 {
                continue;
            }
            indices.push(i as u16);
            indices.push((i + num_vertices_x) as u16);
            indices.push((i + 1) as u16);
            indices.push((i + 1) as u16);
            indices.push((i + num_vertices_x) as u16);
            indices.push((i + num_vertices_x + 1) as u16);
        }
        return indices;
    }

    // fn generate_surface( vertices: &mut Vec<Vertex>) {
    //     let surface_y = 0.0;

    //     for vertex in vertices.iter_mut() {
    //         let y = vertex.position[1];
    //         if y > surface_y {
    //             vertex.color = COLOR7;
    //         } else if y == surface_y {
    //             vertex.color = COLOR2;
    //         } else {
    //             vertex.color = COLOR1;
    //         }
    //         println!("Vertex color: {:?}", vertex.color);
    //     }
    // }
}