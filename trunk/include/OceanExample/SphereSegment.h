#pragma once
#include <osg/Geode>
#include <osg/Geometry>

class SphereSegment : public osg::Geode
{
public:
	SphereSegment( void );
	
	SphereSegment( float radius, 
						unsigned int longitudeSteps, 
						unsigned int lattitudeSteps,
						float longStart,
						float longEnd,
						float latStart,
						float latEnd );

	SphereSegment(const SphereSegment& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

protected:
	~SphereSegment(void);

public:
	// 0 >= longStart/longEnd <= 180
	// 0 >= latStart/latEnd <= 360
	void compute( float radius, 
					  unsigned int longitudeSteps, 
					  unsigned int lattitudeSteps,
					  float longStart,
					  float longEnd,
					  float latStart,
					  float latEnd	);
private:
	osg::Vec2 sphereMap( osg::Vec3& vertex, float radius);

	inline unsigned int idx(unsigned int r, unsigned int c, unsigned int row_len)
	{
		return c + r * row_len;
	}
};
