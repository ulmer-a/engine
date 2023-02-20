#version 330 core

layout (location = 0) in vec3 a_Position;
layout (location = 1) in vec3 a_Normal;
layout (location = 2) in vec3 a_TexCoord;
layout (location = 3) in uvec4 a_BoneIds1;
layout (location = 4) in uvec4 a_BoneIds2;
layout (location = 5) in vec4 a_Weights1;
layout (location = 6) in vec4 a_Weights2;

uniform mat4 u_MVP;
uniform mat4 u_boneTransforms[256];

out float frag_weight;
out vec3 frag_normals;

void main()
{
    mat4 boneTransform =
    u_boneTransforms[a_BoneIds1.x] * a_Weights1.x +
    u_boneTransforms[a_BoneIds1.y] * a_Weights1.y +
    u_boneTransforms[a_BoneIds1.z] * a_Weights1.z +
    u_boneTransforms[a_BoneIds1.w] * a_Weights1.w +
    u_boneTransforms[a_BoneIds2.x] * a_Weights2.x +
    u_boneTransforms[a_BoneIds2.y] * a_Weights2.y +
    u_boneTransforms[a_BoneIds2.z] * a_Weights2.z +
    u_boneTransforms[a_BoneIds2.w] * a_Weights2.w;
    gl_Position = u_MVP * boneTransform * vec4(a_Position, 1.0);

    frag_weight = 1.0;
    frag_normals = a_Normal;
}
