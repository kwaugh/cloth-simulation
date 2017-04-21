R"zzz(#version 330 core
in vec4 vertex_position;
in vec4 vertex_normal;
uniform mat4 view;
uniform vec4 light_position;
out vec4 vs_light_direction;
out vec4 vs_pos;
out vec4 vs_color;
void main()
{
    gl_Position = view * vertex_position;
    vs_light_direction = -gl_Position + view * light_position;
    vs_pos = vertex_position;
    float cosTheta = dot(normalize(vs_light_direction), normalize(vertex_normal));
    cosTheta = clamp(cosTheta, 0.0, 1.0);
    cosTheta = 1.0;
    vs_color = clamp(cosTheta * vec4(1.0, 0.0, 0.0, 1.0), 0.0, 1.0);
}
)zzz"
