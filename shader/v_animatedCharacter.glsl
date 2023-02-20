#version 330 core

layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec3 a_Normal;
layout (location = 2) in vec3 a_TexCoord;
layout (location = 3) in uint a_BoneIds[10];
layout (location = 4) in float a_Weights[10];

uniform mat4 u_MVP;

out vec3 frag_normals;

void main()
{
    frag_normals = a_Normal;
    gl_Position = u_MVP * vec4(a_Position, 1.0);
}
