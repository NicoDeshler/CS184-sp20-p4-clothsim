#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
   vec3 disp2cam = u_cam_pos - v_position.xyz;
   vec3 wo = normalize(disp2cam);
   vec3 wi = reflect(wo, v_normal.xyz);
   out_color = texture(u_texture_cubemap, wi);



  out_color.a = 1;
}
