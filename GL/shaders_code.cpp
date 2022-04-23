#include "shaders_code.h"
const char* shader_simple_v=R"""(
#version 330 core
layout(location = 0) in vec4 position;
layout(location = 1) in vec4 aColor;

out vec4 v_color;

uniform mat4 u_MVP;

void main()
{
  gl_Position = u_MVP * position;
  v_color = aColor;
};
)""";
const char* shader_simple_f=R"""(
#version 330 core
uniform vec4 u_Color;

layout(location = 0) out vec4 color;
in vec4 v_color;

void main()
{
	color = v_color;
};
)""";



const char* shader_pc_intensity_v=R"""(
#version 330 core
layout(location = 0) in vec4 position;
layout(location = 3) in float intensity;

out vec4 v_color;

uniform mat4 u_MVPPC;
uniform vec4 u_COLORPC;


void main()
{
  gl_Position = u_MVPPC * position;
  v_color = u_COLORPC*5*(intensity/255);
};
)""";


const char* shader_pc_intensity_head_v=R"""(
#version 330 core
layout(location = 0) in vec4 position;
layout(location = 1) in float angle;
layout(location = 3) in float intensity;

out vec4 v_color;

uniform mat4 u_MVPPC;
uniform vec4 u_COLORPC;
uniform mat4 u_HEAD;
uniform float u_AngOffset;

void main()
{
  float s = sin(angle);
  float c = cos(angle);
  mat4 rot_angle = mat4(c, -s, 0, 0,
                s,  c, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1);
  vec4 global = rot_angle * u_HEAD * position;
  gl_Position = u_MVPPC * global;
  v_color = u_COLORPC*5*(intensity/255);
};
)""";

const char* shader_pc_intensity_f=R"""(
#version 330 core

layout(location = 0) out vec4 color;
in vec4 v_color;

void main()
{
    color = v_color;
};
)""";


const char* shader_pc_spherical_head_v=R"""(
#version 330 core
#define M_PI 3.1415926535897932384626433832795
#define M_PI2 3.1415926535897932384626433832795/2.0

layout(location = 0) in vec4 position;
layout(location = 1) in float angle;
layout(location = 3) in float intensity;

out vec2 TexCoord;

uniform mat4 u_MVPPC;
uniform vec4 u_COLORPC;
uniform mat4 u_HEAD;
uniform float u_AngOffset;
uniform mat4 u_LADYBUG;
void main()
{
  float s = sin(angle);
  float c = cos(angle);
  mat4 rot_angle = mat4(c, -s, 0, 0,
                s,  c, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1);

  vec4 global_t = rot_angle * u_HEAD * position;
  vec4 global = u_LADYBUG * global_t;
  gl_Position = u_MVPPC * global_t;

  float alpha2 = atan(global.x,global.y);
  float r = sqrt(global.x*global.x + global.y*global.y + global.z*global.z);
  float omega2 = asin(global.z/r);

  TexCoord.x = 0.5+alpha2 / M_PI2;
  TexCoord.y = 0.5+omega2 / M_PI;

};
)""";

const char* shader_pc_spherical_f=R"""(
#version 330 core
uniform vec2 u_texCoordOut;

layout(location = 0) out vec4 color;
in vec2 TexCoord;
uniform sampler2D u_Texture;

void main()
{
	color = texture(u_Texture, TexCoord);
};
)""";