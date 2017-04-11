R"zzz(#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;
uniform mat4 projection;
in vec4 vs_light_direction[];
in vec4 v_pos[];
flat out vec4 normal;
flat out vec4 world_normal;
out vec4 light_direction;
out vec4 world_position;
void main()
{
    int n = 0;
    vec3 ba = vec3(gl_in[1].gl_Position - gl_in[0].gl_Position);
    vec3 cb = vec3(gl_in[2].gl_Position - gl_in[1].gl_Position);
    normal = normalize(vec4(cross(ba, cb), 0.0));

    vec3 world_ba = vec3(v_pos[1] - v_pos[0]);
    vec3 world_cb = vec3(v_pos[2] - v_pos[1]);
    world_normal = normalize(vec4(cross(world_ba, world_cb), 0.0));
    for (n = 0; n < gl_in.length(); n++) {
        light_direction = vs_light_direction[n];
        gl_Position = projection * gl_in[n].gl_Position;
        world_position = v_pos[n];
        EmitVertex();
    }
    EndPrimitive();
}
)zzz"
