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

@vertex
fn vs_main(
    vertex: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.color = instance.color;

    // var len = length(vertex.position); // Not needed as we know the length is 1.0
    var scaled_vs_pos = vertex.position * instance.radius;

    out.clip_position = vec4<f32>(scaled_vs_pos, 1.0) + vec4<f32>(instance.position, 0.0);
    return out;
}

// Fragment shader
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
