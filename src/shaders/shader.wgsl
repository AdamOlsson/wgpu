// Vertex shader
// https://webgpufundamentals.org/webgpu/lessons/webgpu-wgsl-function-reference.html

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct InstanceInput {
    @location(2) position: vec3<f32>,
    @location(3) color: vec3<f32>,
    @location(4) radius: f32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@group(0) @binding(0)
var<uniform> window_size: vec2<f32>;

@vertex
fn vs_main(
    vertex: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = instance.color;

    let scaled_object_position = instance.position / vec3<f32>(window_size, 1.0);

    // Circle vertices are defined with radius 1.0 using vertices
    let scaled_object_radius = vec2<f32>(instance.radius, instance.radius) / window_size;
    let scaled_vertex_position = vertex.position * vec3<f32>(scaled_object_radius, 1.0);

    out.clip_position = vec4<f32>(scaled_vertex_position, 1.0) + vec4<f32>(scaled_object_position, 0.0);
    return out;
}

// Fragment shader
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
