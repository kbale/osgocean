#include <osgOcean/OceanTechnique>

using namespace osgOcean;

OceanTechnique::OceanTechnique(void):
	_isDirty( true )
{

}

OceanTechnique::OceanTechnique( const OceanTechnique& copy, const osg::CopyOp& copyop ):
	osg::Geode		( copy, copyop ),
	_isDirty			( true )
{

}

void OceanTechnique::build(void)
{
	osg::notify(osg::DEBUG_INFO) << "OceanTechnique::build() Not Implemented" << std::endl;
}

void OceanTechnique::stopAnimation(void)
{
	osg::notify(osg::DEBUG_INFO) << "OceanTechnique::stopAnimation() Not Implemented" << std::endl;
}

void OceanTechnique::startAnimation(void)
{
	osg::notify(osg::DEBUG_INFO) << "OceanTechnique::startAnimation() Not Implemented" << std::endl;
}

float OceanTechnique::getSurfaceHeight(void)
{
	osg::notify(osg::DEBUG_INFO) << "OceanTechnique::getSurfaceHeight() Not Implemented" << std::endl;
	return 0.f;
}
