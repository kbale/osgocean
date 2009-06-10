uniform samplerRect uGodRayTexture;

uniform vec3	uSunDir;			
uniform vec3	uHGg;				// Eccentricity constants controls power of forward scattering
uniform float	uIntensity;		// Intensity tweak for god rays
uniform vec3   uEye;

varying vec3 vRay;

const float bias = 0.15; // used to hide aliasing

// Mie phase function
float computeMie(vec3 viewDir, vec3 sunDir) 
{
	float num = uHGg.x;
	float den = (uHGg.y - uHGg.z*dot(sunDir, viewDir)); 
	den = inversesqrt(den); 

	float phase = num * (den*den*den);

	return phase;
}

// ----------------------------------------------
//                Main Program											
// ----------------------------------------------

void main( void )
{
	vec4 shafts;

	// average the pixels out a little to hide aliasing
	// TODO: Maybe try a weak blur filter
	shafts += textureRect(uGodRayTexture, gl_TexCoord[1].xy);
	shafts += textureRect(uGodRayTexture, gl_TexCoord[1].zw);
	shafts += textureRect(uGodRayTexture, gl_TexCoord[2].xy);
	shafts += textureRect(uGodRayTexture, gl_TexCoord[2].zw);

	shafts /= 4.0;

	vec3 rayNormalised = normalize(vRay-uEye);

	float phase = computeMie(rayNormalised, -uSunDir);

	// Calculate final color, adding a little bias (0.15 here)
	// to hide aliasing
	vec3 colour = (bias+uIntensity*shafts.rgb)*phase;

	vec3 ray = ( rayNormalised + vec3(1.0) ) / 2.0; 

	gl_FragColor = vec4(colour, 1.0);
}



