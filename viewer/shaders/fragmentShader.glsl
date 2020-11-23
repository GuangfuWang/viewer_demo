#version 430

out vec4 mcolor;
in vec3 meshColor;

void main(){
    mcolor = vec4(meshColor,1.0);
}