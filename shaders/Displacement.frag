#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
    vec4 H = texture(u_texture_2, uv);
    return H.r;
}

void main() {
 // YOUR CODE HERE
  
  // Get bumped normal vector in world coordinates
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n,t);
  mat3 o2w = mat3(t,b,n);
  float w = u_texture_2_size[0];
  float h = u_texture_2_size[1];
  float k = u_normal_scaling * u_height_scaling;
  float dU = 10*(h(v_uv + vec2(1.0/w , 0.0)) - h(v_uv)) * k;
  float dV = 10*(h(v_uv + vec2(0.0 , 1.0/h)) - h(v_uv)) * k;
  vec3 n0 = vec3(-dU, -dV, 1);
  vec3 nd = o2w * normalize(n0);   // bumped normal vector
  

  // Blinn Phong Shading
  // Note, we now use the bumped normal vector 'nd' as the normal vector for shading

  // Adjustable params
  float ka = 0.1;
  float kd = 1;
  float ks = 0.8;
  float Ia = 0.1;
  float p = 50.0;
  
  // Phong shading
  vec3 disp2light = u_light_pos - v_position.xyz;
  vec3 disp2cam = u_cam_pos - v_position.xyz;
  vec3 incident = normalize(disp2light);
  vec3 outgoing = normalize(disp2cam);
  vec3 halfvec =  normalize(incident + outgoing);
  
  float r = length(disp2light);
  float r2 = r*r;
  
  // weights
  float wd = max(dot(nd, incident), 0.0) / r2;
  float ws = pow(max(dot(nd, halfvec), 0.0), p) / r2;
  vec3 color = (ka * Ia) + (kd * wd + ks * ws) * u_light_intensity;
  
  // Output color with alpha blending factor 
  out_color = vec4(color, 0.0);
  out_color.a = 1.0;}

