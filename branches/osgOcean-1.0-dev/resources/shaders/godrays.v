const int NUM_WAVES = 16;

uniform vec3 uOrigin;						// central position of vertices - sun position on water surface
uniform vec3 uExtinction_c;				// extinction coefficient
uniform vec3 uEye;							// Eye position in world space
uniform vec3 uSunDir;						// sunlight direction
uniform float uSpacing;						// spacing between vertices
uniform float uWaves[NUM_WAVES * 5];	// wave constants

varying vec3 vIntensity;

float fastFresnel(vec3 I, vec3 N, float r0) 
{
	return r0 + (1.0-r0) * pow(1.0-dot(I, N), 5.0);
}

vec3 calculateWaterNormal(float x0, float y0) 
{
	vec3 t1 = vec3(1.0,0.0,0.0);
	vec3 t2 = vec3(0.0,1.0,0.0);

	int itr = NUM_WAVES/4;

	for (int i = 0, j = 0; i < itr; i++, j += 20)
	{
		vec4 kx    = vec4( uWaves[j+0],  uWaves[j+1],  uWaves[j+2],  uWaves[j+3] );
		vec4 ky    = vec4( uWaves[j+4],  uWaves[j+5],  uWaves[j+6],  uWaves[j+7] );
		vec4 Ainvk = vec4( uWaves[j+8],  uWaves[j+9],  uWaves[j+10], uWaves[j+11] );
		vec4 A     = vec4( uWaves[j+12], uWaves[j+13], uWaves[j+14], uWaves[j+15] );
		vec4 wt    = vec4( uWaves[j+16], uWaves[j+17], uWaves[j+18], uWaves[j+19] );
		vec4 phase = (kx*x0 + ky*y0 - wt);
		vec4 sinp, cosp;
		sincos(phase, sinp, cosp);

		// Update tangent vector along x0
		t1.x -= dot(Ainvk, kx*cosp*kx);
		t1.y -= dot(Ainvk, ky*cosp*kx);
		t1.z += dot(A, (-sinp)*(kx));

		// Update tangent vector along y0
		t2.x -= dot(Ainvk, kx*cosp*ky);
		t2.y -= dot(Ainvk, ky*cosp*ky);
		t2.z += dot(A, (-sinp)*(ky));
	}

	// Calculate and return normal
	return normalize( cross(t1, t2) ); 
}

// ----------------------------------------------
//                Main Program											
// ----------------------------------------------

void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0;

	// Scale and translate the vertex on the water surface
	vec3 worldPos = gl_Vertex.xyz * vec3(uSpacing,uSpacing,1.0);
	worldPos += uOrigin;

	// Calculate the water normal at this point
	vec3 normal = calculateWaterNormal(worldPos.x, worldPos.y);

	// Calculate transmittance 
	// BUG: makes intensity too small not sure why.
	float transmittance = 1.0-fastFresnel(-uSunDir, normal, 0.0204);

	// Extrude bottom vertices along the direction of the refracted 
	// sunlight
	if (gl_TexCoord[0].s > 0.0) 
	{
		// Calculate refraction vector and extrude polygon
		vec3 refr = refract(uSunDir, normal, 0.75);//*transmittance;
		worldPos += refr*gl_TexCoord[0].s;
		// intensity to zero
		vIntensity = vec3(0.0);	
	}
	//else
	//{
		// Set intensity so that the further away you go from the surface
		float totalDist = gl_TexCoord[0].s + length(worldPos-uEye);
		vIntensity = exp(-totalDist*uExtinction_c)*transmittance;
		vIntensity = clamp(vIntensity, 0.0,  0.06);
	//}

	// Transform position from world to clip space
	gl_Position = gl_ModelViewProjectionMatrix * vec4(worldPos, 1.0 );
	// Tweak z position not to clip shafts very close to the viewer
	gl_Position.z = 0.01;
}