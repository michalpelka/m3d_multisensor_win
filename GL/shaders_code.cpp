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

  gl_Position = u_MVPPC * rot_angle * u_HEAD * position;
  v_color = u_COLORPC*5*(intensity/255);
};
)""";

const char* shader_pc_intensity_f=R"""(
#version 330 core
uniform vec4 u_Color;

layout(location = 0) out vec4 color;
in vec4 v_color;

void main()
{
	color = v_color;
};
)""";