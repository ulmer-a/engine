#version 330 core

in float frag_weight;
in vec3 frag_normals;
out vec4 fragColor;


void main()
{
    fragColor = vec4((frag_normals * frag_weight), 1.0);
}
