
@group(0) @binding(0) var texture_sampler: sampler; 
@group(1) @binding(0) var input_texture: texture_2d<f32>;
// TODO: Unsure about the type of the texture_storage
@group(1) @binding(1) var output_texture: texture_storage_2d<bgra8unorm, write>;

@compute @workgroup_size(32,1,1)
fn cs_main(
    @builtin(workgroup_id) workgroup_id: vec3u,
    @builtin(local_invocation_id) local_invocation_id: vec3u
) {
    let tile_size = vec2i(32,32);

    let work_group_base_idx = vec2i(workgroup_id.xy * vec2(1,32));
    let local_invocation_base_idx = vec2i(local_invocation_id.xy * vec2(32,1));
    let base_idx = work_group_base_idx + local_invocation_base_idx;  
    
    for (var r = 0; r < tile_size.x; r++) {
        for (var c = 0; c < tile_size.y; c++) {
            let sample_idx = base_idx + vec2(r,c);
            let s = textureSampleLevel(input_texture, texture_sampler, vec2f(sample_idx), 0.0);
            textureStore(output_texture, sample_idx, s);
        }
    }
}

