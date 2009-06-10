// osgOcean Uniforms
// -----------------
uniform int uLightID;
uniform mat4 osg_ViewMatrixInverse;
// -----------------

varying vec3 vNormal;
varying vec3 vLightDir;
varying vec3 vEyeVec;
varying float vWorldHeight;

void main(void)
{
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = ftransform();
	gl_FogFragCoord = gl_Position.z;
	gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex; // for reflections

	vNormal = gl_NormalMatrix * gl_Normal;
	vLightDir = gl_LightSource[uLightID].position.xyz;
	vEyeVec = -vec3(gl_ModelViewMatrix*gl_Vertex);
	
	vec4 worldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;

	vWorldHeight = worldVertex.z;
}