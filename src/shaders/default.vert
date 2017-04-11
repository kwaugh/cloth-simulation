R"zzz(#version 330 core
in vec4 vertex_position;
uniform mat4 view;
uniform vec4 light_position;
out vec4 vs_light_direction;
out vec4 v_pos;
void main()
{
    gl_Position = view * vertex_position;
    vs_light_direction = -gl_Position + view * light_position;
    v_pos = vertex_position;
}
)zzz"
