varying vec4 vEyePos;

void main(void) 
{
	vEyePos = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_Position = ftransform();
}