R"zzz(#version 330 core
flat in vec4 normal;
flat in vec4 world_normal;
in vec4 light_direction;
in vec4 interp_color;
out vec4 fragment_color;
void main()
{
    vec4 norm_normal = normalize(normal);
    vec4 color;
    if (gl_FrontFacing) {
        color = vec4(0.0, 1.0, 0.0, 1.0);
    } else {
        color = vec4(1.0, 0.0, 0.0, 1.0);
    }
    float dot_nl = dot(normalize(light_direction), norm_normal);
    dot_nl = clamp(dot_nl, 0.0, 1.0);
    /* fragment_color = clamp(dot_nl * color, 0.0, 1.0); */
    fragment_color = interp_color;
}
)zzz"
