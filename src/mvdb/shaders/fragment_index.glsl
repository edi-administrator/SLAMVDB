#version 330 core
flat in int v_voxel_index;

out int frag_index;

void main() {
    frag_index = v_voxel_index;
}
