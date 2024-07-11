struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) tex_coord: vec3<f32>,
};

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) tex_coord: vec3<f32>
) -> VertexOut {
    var output: VertexOut;
    output.position = vec4<f32>(position, 1.0);
    output.tex_coord = tex_coord;
    return output;
}

@group(0) @binding(0) var gray_sampler: sampler;
@group(0) @binding(1) var input_texture: texture_2d<f32>;
@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    let texel = textureSample(input_texture, gray_sampler, in.tex_coord.xy);
    let gray = (texel.r + texel.g + texel.b) / 3.0;
    return vec4(gray, gray, gray, 1.0);
}