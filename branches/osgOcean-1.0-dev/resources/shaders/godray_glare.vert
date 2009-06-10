uniform vec3 uOrigin;
uniform vec3 uExtinction_c;
uniform vec3 uEye;
uniform float uSpacing;

varying vec3 vIntensity;

// ----------------------------------------------
//                Main Program											
// ----------------------------------------------

void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0;

	vec3 worldPos = gl_Vertex.xyz * vec3(uSpacing,uSpacing,1.0);
	worldPos += uOrigin;

	vec3 extinct = vec3(0.2,0.2,0.2);

	float totalDist = length(worldPos-uEye)/3.0;
	vIntensity = exp(-totalDist*uExtinction_c);
	vIntensity = clamp(vIntensity, 0.0,  1.0);

	gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos,1.0);
}