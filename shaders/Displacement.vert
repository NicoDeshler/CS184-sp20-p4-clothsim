#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  vec4 H = texture(u_texture_2, uv);
  return H.r;
}

void main() {
  
  v_normal = normalize(u_model * in_normal);
  v_tangent = normalize(u_model * in_tangent);
  v_uv = in_uv;

  // Displacement vector
  vec4 disp = h(v_uv) * v_normal * u_height_scaling;
  v_position = u_model * in_position + disp;
  gl_Position = u_view_projection * v_position;
  
  
}
