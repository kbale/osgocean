#include <osgOcean/MipmapGeometry>

namespace osgOcean
{
	MipmapGeometry::MipmapGeometry( void ):
		_level			( 0 ),
		_numLevels		( 0 ),
		_resolution		( 0 ),
		_rowLen			( 0 ),
		_colLen			( 0 ),
		_startIdx		( 0 ),
		_border			( BORDER_NONE )
	{

	}

	MipmapGeometry::MipmapGeometry(	unsigned int level, 
												unsigned int numLevels, 
												unsigned int startIdx,
												BORDER_TYPE border ):
		_level			( level ),
		_numLevels		( numLevels ),
		_resolution		( 2 << (numLevels-(level+1)) ),
		_rowLen			( border==BORDER_X || border==BORDER_XY ? _resolution+1 : _resolution),
		_colLen			( border==BORDER_Y || border==BORDER_XY ? _resolution+1 : _resolution),
		_startIdx		( startIdx ),
		_border			( border )
	{
	}

	MipmapGeometry::~MipmapGeometry( void )
	{

	}

	MipmapGeometry::MipmapGeometry( const MipmapGeometry& copy, const osg::CopyOp& copyop ):
		osg::Geometry	( copy, copyop ),
		_level			( copy._level ),
		_numLevels		( copy._numLevels ),
		_resolution		( copy._resolution ),
		_rowLen			( copy._rowLen ),
		_colLen			( copy._colLen ),
		_startIdx		( copy._startIdx ),
		_border			( copy._border )
	{
		
	}

	//void MipmapGeometry::apply(osg::State& state )
	//{
	//	if( state.setActiveTextureUnit(3) )
	//	{
	//		glMatrixMode(GL_TEXTURE);
	//		glLoadIdentity();
	//		float noisetilescale = 32.0f;//meters (128/16=8, 8tex/m).
	//		// fixme 32m wide noise is a good compromise, that would be a noise
	//		// sample every 12.5cm. But it works only if the noise map
	//		// has high frequencies in it... this noise map has to few details
	//		glScalef(4,4,1);	// fixme adjust scale
	//		//glTranslatef(transl.x, transl.y, 0);
	//		glMatrixMode(GL_MODELVIEW);
	//	}
	//	else
	//	{
	//		osg::notify(osg::NOTICE) << "fail" << std::endl;
	//	}

	//}
}