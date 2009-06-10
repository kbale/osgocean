uniform samplerRect uTexture;

void main( void )
{
   vec2 texCoordSample = vec2( 0.0 );
   
   vec4 color = 0.5 * textureRect( uTexture, gl_TexCoord[0] );
   
   texCoordSample.x = gl_TexCoord[0].x;
   texCoordSample.y = gl_TexCoord[0].y + 1;
   color += 0.25 * textureRect( uTexture, texCoordSample);
   
   texCoordSample.y = gl_TexCoord[0].y - 1;
   color += 0.25 * textureRect( uTexture, texCoordSample);

   gl_FragColor = color;
}