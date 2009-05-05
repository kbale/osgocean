#pragma once
#include "SphereSegment.h"
#include <osg/Program>
#include <osg/Uniform>
#include <osg/TextureCubeMap>

class SkyDome : public SphereSegment
{
private:
	enum {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

public:
	SkyDome( void );
	SkyDome( const SkyDome& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
	SkyDome( float radius, unsigned int longSteps, unsigned int latSteps, osg::TextureCubeMap* cubemap );

protected:
	~SkyDome(void);
	
public:
	void setupStateSet( osg::TextureCubeMap* cubemap );
	void create( float radius, unsigned int latSteps, unsigned int longSteps, osg::TextureCubeMap* cubemap );
	
	inline void setCubeMap( osg::TextureCubeMap* cubemap ){
		getOrCreateStateSet()->setTextureAttributeAndModes( 0, cubemap, osg::StateAttribute::ON );
	}

private:
	osg::ref_ptr<osg::Program> createShader(void);

};
