
struct VertexInput {
    @location(0) position: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) pos: vec4<f32>
}

@vertex
fn vs_main(
    vertex: VertexInput,
) -> VertexOutput {

    // Vertices will be:
    // [  1.0,  1.0 ]
    // [  1.0, -1.0 ]
    // [ -1.0, -1.0 ]
    // [  1.0,  1.0 ]
    // [ -1.0, -1.0 ]
    // [ -1.0,  1.0 ]
    var out: VertexOutput;
    out.pos = vec4<f32>(vertex.position, 0.0);
    return out;
}

@group(0) @binding(0) var bloom_sampler: sampler;
@group(1) @binding(0) var input_texture: texture_2d<f32>; 
@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
   return textureSample(input_texture, bloom_sampler, in.pos.xy);
}