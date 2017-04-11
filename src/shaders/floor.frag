R"zzz(#version 330 core
flat in vec4 normal;
in vec4 light_direction;
in vec4 world_position;
out vec4 fragment_color;
void main()
{
	float dot_nl = dot(normalize(light_direction), normal);
    float PI = 3.1415926535897932384626433832795;
	dot_nl = clamp(dot_nl, 0.0, 1.0);
    /* figure out the checkerboard color */
    if (sin(PI * world_position.x) * sin(PI * world_position.z) > 0) {
        fragment_color = clamp(dot_nl * vec4(0.0, 0.0, 0.0, 1.0), 0.0, 1.0);
    } else {
        fragment_color = clamp(dot_nl * vec4(1.0, 1.0, 1.0, 1.0), 0.0, 1.0);
    }
}
)zzz"
