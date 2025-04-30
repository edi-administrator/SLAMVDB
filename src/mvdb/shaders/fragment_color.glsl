#version 330 core
flat in vec4 v_voxel_color;

out vec4 frag_color;

void main() {
    frag_color = v_voxel_color;
}
