uniform samplerRect uTexture;

void main( void )
{
	vec2 texCoordSample = vec2(0.0);

	texCoordSample.x = gl_TexCoord[0].x - 1;
	texCoordSample.y = gl_TexCoord[0].y + 1;
	vec4 color = textureRect(uTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x + 1;
	texCoordSample.y = gl_TexCoord[0].y + 1;
	color += textureRect(uTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x + 1;
	texCoordSample.y = gl_TexCoord[0].y - 1;
	color += textureRect(uTexture, texCoordSample);

	texCoordSample.x = gl_TexCoord[0].x - 1;
	texCoordSample.y = gl_TexCoord[0].y - 1;
	color += textureRect(uTexture, texCoordSample);

	gl_FragColor = color * 0.25;
}