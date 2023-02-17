#version 330 core

layout (location = 0) in vec3 a_Position;
uniform mat4 u_MVP;
//out vec4 triangleColor;

void main()
{
    gl_Position = u_MVP * vec4(a_Position, 1.0);

    //    if ((int)(a_Position.x) % 2 == 0 && (int)(a_Position.y) % 2 == 0)
    //
    //    if (color < 0) {
    //        triangleColor = vec4(0.0, 1.0, 0.0, 1.0);
    //    } else {
    //        triangleColor = vec4(0.2, 0.5, 0.2, 1.0);
    //    }
}
