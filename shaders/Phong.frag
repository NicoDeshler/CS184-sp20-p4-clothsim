#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {

  // Adjustable params
  /*
  //ONLY AMBIENT
  float ka = 0.5;
  float kd = 0.0;
  float ks = 0.0;
  float Ia = 1.0;
  float p = 0.0;
   
  //ONLY DIFFUSE 
  float ka = 0.0;
  float kd = 1.0;
  float ks = 0.0;
  float Ia = 0.0;
  float p = 0.0;
  
  //ONLY SPECULAR
  float ka = 0.0;
  float kd = 0.0;
  float ks = 0.8;
  float Ia = 0.0;
  float p = 50.0;
  */
  // FULL PHONG
  float ka = 0.1;
  float kd = 1.0;
  float ks = 0.8;
  float Ia = 0.1;
  float p = 50.0;
  /**/




  
  // Phong shading
  vec3 disp2light = u_light_pos - v_position.xyz;
  vec3 disp2cam = u_cam_pos - v_position.xyz;
  vec3 incident = normalize(disp2light);
  vec3 outgoing = normalize(disp2cam);
  vec3 halfvec =  normalize(incident + outgoing);
  
  float r = length(disp2light);
  float r2 = r*r;
  // weights
  float wd = max(dot(v_normal.xyz, incident), 0.0) / r2;
  float ws = pow(max(dot(v_normal.xyz, halfvec), 0.0), p) / r2;
  vec3 color = ka * Ia + (kd * wd + ks * ws) * u_light_intensity;
  
  // Output color with alpha blending factor 
  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}

