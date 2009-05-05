uniform mat4 osg_ViewMatrixInverse;

uniform int uLightID;

uniform vec3 uEyePosition;

uniform vec3 uNoiseCoords0;
uniform vec3 uNoiseCoords1;

uniform float uFogDensity;

uniform vec4 uWaveTop;
uniform vec4 uWaveBot;

varying vec4 vVertex;
varying vec3 vNormal;
varying vec3 vViewerDir;
varying vec3 vLightDir;

varying float vFogFactor;

// world space variables
varying vec3 vWorldViewDir;
varying vec3 vWorldNormal;

mat3 get3x3Matrix( mat4 m )
{
	mat3 result;
	
	result[0][0] = m[0][0]; 
	result[0][1] = m[0][1]; 
	result[0][2] = m[0][2]; 

	result[1][0] = m[1][0]; 
	result[1][1] = m[1][1]; 
	result[1][2] = m[1][2]; 
	
	result[2][0] = m[2][0]; 
	result[2][1] = m[2][1]; 
	result[2][2] = m[2][2]; 
	
	return result;
}

float calcFogFactor( float fog_frag, float fogDensity)
{
	const float LOG2 = 1.442695;
	float fog = exp2( -fogDensity * fogDensity * fog_frag * fog_frag * LOG2 );

	return clamp( fog, 0.0, 1.0 );
}

void projCoords( int texUnit )
{
    vec4 ecPosition = gl_ModelViewMatrix * gl_Vertex;
    gl_TexCoord[texUnit].s = dot( ecPosition, gl_EyePlaneS[texUnit] );
    gl_TexCoord[texUnit].t = dot( ecPosition, gl_EyePlaneT[texUnit] );
    gl_TexCoord[texUnit].p = dot( ecPosition, gl_EyePlaneR[texUnit] );
    gl_TexCoord[texUnit].q = dot( ecPosition, gl_EyePlaneQ[texUnit] );
}

// -------------------------------
//          Main Program
// -------------------------------

void main( void )
{
	gl_Position = ftransform();

	mat4 worldObjectMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;
	vec4 worldVertex = worldObjectMatrix * gl_Vertex;

	// -----------------------------------------------------------

	vVertex = gl_Vertex;

	vec4 v = gl_ModelViewMatrix * gl_Vertex;

	// Light direction in OBJECT space
	vLightDir = normalize( vec3( gl_ModelViewMatrixInverse * ( gl_LightSource[uLightID].position - v ) ) );
	
	// Viewer direction in OBJECT space
	vViewerDir = gl_ModelViewMatrixInverse[3].xyz - gl_Vertex.xyz;

	vNormal = normalize(gl_Normal);

	vec4 upwelltopbot = uWaveTop-uWaveBot;
	
	float val =  gl_Vertex.z * 0.1111111 + vNormal.z - 0.4666667;

	gl_FrontColor = upwelltopbot *
		clamp((gl_Vertex.z + uEyePosition.z) * 0.1111111 + vNormal.z - 0.4666667, 0.0, 1.0) + uWaveBot;

	// -------------------------------------------------------------

	mat3 ModelWorld3x3 = get3x3Matrix( worldObjectMatrix );
	
	// find world space normal.
	vWorldNormal = ModelWorld3x3 * gl_Normal; 
	
	// find world space eye vector.
	vWorldViewDir = worldVertex.xyz - uEyePosition.xyz;	

	// ------------- Texture Coords ---------------------------------

	// Normal Map Coords
	gl_TexCoord[2].xy = ( gl_Vertex.xy * uNoiseCoords0.z + uNoiseCoords0.xy );
	gl_TexCoord[2].zw = ( gl_Vertex.xy * uNoiseCoords1.z + uNoiseCoords1.xy );

	// Foam coords
	gl_TexCoord[0] = gl_MultiTexCoord0 * 40.0;

	// ------------- Fog Coords --------------------------------------
	gl_FogFragCoord = gl_Position.z;

	vFogFactor = calcFogFactor( gl_Position.z, uFogDensity );
}