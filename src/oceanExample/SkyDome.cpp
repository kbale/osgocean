#include "SkyDome.h"

SkyDome::SkyDome( void )
{
	
}

SkyDome::SkyDome( const SkyDome& copy, const osg::CopyOp& copyop ):
	SphereSegment( copy, copyop )
{

}

SkyDome::SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap )
{
	compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
	setupStateSet(cubemap);
}

SkyDome::~SkyDome(void)
{
}

void SkyDome::create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap )
{
	compute( radius, longSteps, latSteps, 90.f, 180.f, 0.f, 360.f );
	setupStateSet(cubemap);
}

void SkyDome::setupStateSet( osg::TextureCubeMap* cubemap )
{
	osg::StateSet* ss = new osg::StateSet;

	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
	ss->setAttributeAndModes( createShader().get(), osg::StateAttribute::ON );
	ss->addUniform( new osg::Uniform("uEnvironmentMap", 0) );

	setStateSet(ss);
}

osg::ref_ptr<osg::Program> SkyDome::createShader(void)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;

	char vertexSource[]=
		"varying vec3 vTexCoord;\n"
		"\n"
		"void main(void)\n"
		"{\n"
		"	gl_Position = ftransform();\n"
		"	vTexCoord = gl_Vertex.xyz;\n"
		"}\n";

	char fragmentSource[]=
		"uniform samplerCube uEnvironmentMap;\n"
		"varying vec3 vTexCoord;\n"
		"\n"
		"void main(void)\n"
		"{\n"
		"	vec3 tex = vec3(vTexCoord.x, vTexCoord.y, -vTexCoord.z);\n"
		"	gl_FragColor = textureCube( uEnvironmentMap, tex.xzy );\n"
		"}\n";

	program->setName( "sky_dome_shader" );
	program->addShader(new osg::Shader(osg::Shader::VERTEX,   vertexSource));
	program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentSource));

	return program;
}


