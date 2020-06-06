#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 kd = vec3(1.0,1.0,1.0);
  vec3 disp2light = u_light_pos - v_position.xyz;
  vec3 incident = normalize(disp2light);
  float r = length(disp2light);
  float r2 = r*r;
  float weight = max(dot(v_normal.xyz,incident), 0.0) / r2;
  vec3 color = kd * u_light_intensity * weight;
  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}
