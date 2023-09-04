#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;
in vec3 fragViewPos;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

void main()
{
	vec4 texelColor = texture(texture0, fragTexCoord);
	vec3 normal = normalize(fragNormal);
	float bl = pow(0.5 * (1 + dot(normal, normalize(vec3(1, 8 , 3)))), 2);
	float l = 0.5 + 0.5 * bl;
	finalColor = texelColor*colDiffuse*fragColor;
	finalColor.rgb *= l;
	float fog = exp(-0.009 * length(fragViewPos));
	finalColor.rgb = mix(vec3(0.5, 0.5, 0.8), finalColor.rgb, fog);
}
