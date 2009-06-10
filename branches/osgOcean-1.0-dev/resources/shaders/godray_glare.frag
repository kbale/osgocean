uniform sampler2D uGlareTexture;

varying vec3 vIntensity;

// ----------------------------------------------
//                Main Program											
// ----------------------------------------------

void main(void)
{
	vec3 color = texture2D( uGlareTexture, gl_TexCoord[0].st ).rgb;

	gl_FragColor = vec4((vIntensity*color.r)*1.5, 1.0 );
}