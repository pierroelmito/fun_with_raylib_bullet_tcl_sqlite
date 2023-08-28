#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

out vec2 fragTexCoord;
out vec4 fragColor;
out vec3 fragNormal;
out vec3 fragViewPos;

uniform mat4 mvp;
uniform mat4 matNormal;
uniform mat4 matModel;
uniform mat4 matView;

vec3 fix(vec4 v)
{
	return v.xyz / v.w; 
}

void main()
{
	fragTexCoord = vertexTexCoord;
	fragColor = vertexColor;
	fragNormal = normalize(vec3(matNormal*vec4(vertexNormal, 1.0)));
	fragViewPos = fix((matView * matModel) * vec4(vertexPosition, 1.0));
	gl_Position = mvp * vec4(vertexPosition, 1.0);
}
