varying vec3 vLightDir;
varying vec3 vEyeVec;
varying float vWorldHeight;
varying float unitHeight;

attribute vec3 aTangent;

uniform vec2 uHeightRange;
uniform float uOneOverHeight;
uniform mat4 osg_ViewMatrixInverse;

void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0*vec4(16.0,16.0,1.0,1.0);
	gl_TexCoord[1] = gl_MultiTexCoord0*vec4(8.0,8.0,1.0,1.0);
	gl_TexCoord[2] = gl_MultiTexCoord0*vec4(25.0,25.0,1.0,1.0);
	
	vec3 vertex = vec3(gl_ModelViewMatrix * gl_Vertex);
	vEyeVec = -vertex;

	vec3 n = normalize(gl_NormalMatrix * gl_Normal);
	vec3 t = normalize(gl_NormalMatrix * aTangent);
	vec3 b = cross(n, t);

	vec3 tmpVec = vec3(gl_LightSource[0].position.xyz);
	tmpVec = normalize(tmpVec);

	vLightDir.x = dot(tmpVec, t);
	vLightDir.y = dot(tmpVec, b);
	vLightDir.z = dot(tmpVec, n);

	tmpVec = -vertex;
	vEyeVec.x = dot(tmpVec, t);
	vEyeVec.y = dot(tmpVec, b);
	vEyeVec.z = dot(tmpVec, n);

	gl_Position = ftransform();

	gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex;

	gl_FogFragCoord = gl_Position.z;

	float inv = 1.0 / ( uHeightRange.y - (uHeightRange.x+65.0) );
	unitHeight = inv * (gl_Vertex.z - (uHeightRange.x+65.0));

	vec4 worldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;
	vWorldHeight = worldVertex.z;

	gl_Position = ftransform();
}