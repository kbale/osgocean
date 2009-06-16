uniform samplerRect osgOcean_GaussianTexture;

void main( void )
{
   vec2 texCoordSample = vec2( 0.0 );

   vec4 color = 0.5 * textureRect(osgOcean_GaussianTexture, gl_TexCoord[0] );

   texCoordSample.y = gl_TexCoord[0].y;
   texCoordSample.x = gl_TexCoord[0].x + 1;
   color += 0.25 * textureRect(osgOcean_GaussianTexture, texCoordSample);

   texCoordSample.x = gl_TexCoord[0].x - 1;
   color += 0.25 * textureRect(osgOcean_GaussianTexture, texCoordSample);

   gl_FragColor = color;
}