uniform vec2 uScreenRes;
uniform vec2 uLowRes;

void main( void )
{
	gl_TexCoord[0] = gl_MultiTexCoord0 * vec4( uScreenRes, 1.0, 1.0 );
	gl_TexCoord[1] = gl_MultiTexCoord0 * vec4( uLowRes,    1.0, 1.0 );

	gl_Position = ftransform();
}

