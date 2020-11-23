#version 430

in layout(location=0) vec3 position;
in layout(location=1) vec3 theColor;

uniform mat4 fullTM;

out vec3 meshColor;

void main(){
    vec4 v=vec4(position,1.0);
    gl_Position = fullTM*v;
    meshColor = theColor;
}