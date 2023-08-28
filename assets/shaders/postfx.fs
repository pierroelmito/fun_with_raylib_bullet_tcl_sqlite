#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

void main()
{
	vec2 v = abs(fragTexCoord * 2 - 1);
	float r = 0.4 * length(pow(v, vec2(2, 2)));
	vec4 texelColor = texture(texture0, fragTexCoord);
	finalColor = texelColor*colDiffuse*fragColor;
	finalColor.rgb = mix(finalColor.rgb, vec3(0, 0, 0), r);
}

