uniform samplerCube uEnvironmentMap;

varying vec3 vTexCoord;

void main(void)
{
   vec3 tex = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);
   gl_FragColor = textureCube( uEnvironmentMap, tex.xzy );
}