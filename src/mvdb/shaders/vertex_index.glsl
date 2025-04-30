#version 330 core
layout (location = 0) in vec3 a_pos;
layout (location = 1) in vec3 a_voxel_position;
layout (location = 2) in int a_voxel_index;

uniform mat4 u_projection;
uniform mat4 u_view;
uniform float k1;
uniform float k2;
uniform float p1;
uniform float p2;

uniform float fx;
uniform float cx;
uniform float fy;
uniform float cy;
uniform float _w;
uniform float _h;

flat out int v_voxel_index;

void main() {

    vec4 transformed_voxpos = u_view * vec4(a_voxel_position, 1.0);

    if ( transformed_voxpos[2] < -.5 )
    {
        vec3 transformed_position = a_pos + a_voxel_position;
        vec4 vertex_pos =  u_projection * u_view * vec4(transformed_position, 1.0);

        float w = vertex_pos[3];

        float x_ndc = vertex_pos[0] / w;
        float y_ndc = vertex_pos[1] / w;

        float x_nim = ( 1 / fx ) * ( ( 1 - x_ndc ) * _w / 2 - cx );
        float y_nim = ( 1 / fy ) * ( ( 1 - y_ndc ) * _h / 2 - cy );

        float radius = sqrt( pow( x_nim, 2 ) + pow( y_nim, 2 ) );

        float x_ndc_dst = x_ndc;
        float y_ndc_dst = y_ndc;

        float dr = ( 1 +  pow( radius, 2 ) * k1 + pow( radius, 4 ) * k2 );

        float dt_x = ( 2 * p1 * x_nim * y_nim + p2 * ( pow( radius, 2 ) + 2 * pow( x_nim, 2 ) ) );
        float dt_y = ( 2 * p2 * x_nim * y_nim + p1 * ( pow( radius, 2 ) + 2 * pow( y_nim, 2 ) ) );

        float x_dt = x_nim * dr + dt_x;
        float y_dt = y_nim * dr + dt_y;

        if ( radius < sqrt(2) )
        {
            x_ndc_dst = 1 - 2 * ( x_dt * fx + cx ) / _w;
            y_ndc_dst = 1 - 2 * ( y_dt * fy + cy ) / _h;
        }

        vertex_pos[0] = x_ndc_dst * w;
        vertex_pos[1] = y_ndc_dst * w;
        vertex_pos[2] = vertex_pos[2];
        vertex_pos[3] = w;

        v_voxel_index = int(a_voxel_index);
        gl_Position = vertex_pos;
    }
    else
    {
        gl_Position = vec4(0, 0, 1, 1.0);
        v_voxel_index = 0;
    }
}