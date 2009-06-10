#define NUM_SAMPLES 4

uniform samplerRect uBuffer;
uniform vec2 uDirection;
uniform float uAttenuation;
uniform float uPass;

void main(void)
{
	vec2 sampleCoord = vec2(0.0);
	vec3 cOut = vec3(0.0);
	
	// sample weight = a^(b*s)
	// a = attenuation
	// b = 4^(pass-1)
	// s = sample number
	
	vec2 pxSize = vec2(0.5);

	float b = pow( float(NUM_SAMPLES), float(uPass));
	float sf = 0.0;

	for (int s = 0; s < NUM_SAMPLES; s++)
	{
		sf = float(s);
		float weight = pow(uAttenuation, b * sf);
		sampleCoord = gl_TexCoord[0].st + (uDirection * b * vec2(sf) * pxSize);
		cOut += clamp(weight,0.0,1.0) * textureRect(uBuffer, sampleCoord).rgb;
	}
	
	vec3 streak = clamp(cOut, 0.0, 1.0);

	gl_FragColor = vec4(streak,1.0);
}