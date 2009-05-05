#include <osgOcean/OceanTile>

using namespace osgOcean;

OceanTile::OceanTile( void ):
	_resolution	   (0),
	_rowLength	   (0),
	_numVertices   (0),
	_spacing		   (0),
	_maxDelta	   (0),
	_averageHeight (0)
{}

OceanTile::OceanTile( osg::FloatArray* heights, 
							 unsigned int resolution, 
							 const float spacing ):
	_resolution		( resolution ),
	_rowLength		( _resolution + 1 ),
	_numVertices	( _rowLength*_rowLength ),
	_vertices		( new osg::Vec3Array ),
	_normals			( new osg::Vec3Array(_numVertices) ),
	_spacing			( spacing ),
	_maxDelta		( 0.f ),
	_averageHeight ( 0.f )
{
	_vertices->reserve( _numVertices );

	float x1,y1;
	osg::Vec3f v;

	for(int y = 0; y <= (int)_resolution; ++y )
	{
		y1 = y % _resolution;

		for(int x = 0; x <= (int)_resolution; ++x )
		{
			x1 = x % _resolution;

			unsigned int ptr = array_pos(x1,y1,_resolution);

			v.z() = heights->at( ptr );

			_vertices->push_back( v );
		}
	}

	computeNormals();
	_normalMap = createNormalMap();
	//computeMaxDelta();
}

OceanTile::OceanTile( osg::FloatArray* heights, 
							 osg::Vec2Array* displacements, 
							 unsigned int resolution, 
							 const float spacing ):

	_resolution	( resolution ),
	_rowLength	( _resolution + 1 ),
	_numVertices( _rowLength*_rowLength ),
	_vertices	( new osg::Vec3Array ),
	_normals		( new osg::Vec3Array(_numVertices) ),
	_spacing		( spacing ),
	_maxDelta	( 0.f )
{
	_vertices->reserve( _numVertices );

	float x1,y1;
	osg::Vec3f v;

	float sumHeights = 0.f;

	//debug
	//static int count = 0;
	//std::stringstream ss;
	//ss << "Tile_" << count << ".txt";
	//std::ofstream outFile( ss.str().c_str() );
	//outFile << _rowLength << std::endl;
	//++count;

	for(int y = 0; y <= (int)_resolution; ++y )
	{
		y1 = y % _resolution;

		for(int x = 0; x <= (int)_resolution; ++x )
		{
			x1 = x % _resolution;

			unsigned int ptr = array_pos(x1,y1,_resolution);

			v.x() = displacements->at(ptr).x();
			v.y() = displacements->at(ptr).y();
			v.z() = heights->at( ptr );

			// debug
			//outFile << v.x() << std::endl;
			//outFile << v.y() << std::endl;
			//outFile << v.z() << std::endl;

			sumHeights += v.z();

			_vertices->push_back( v );
		}
	}

	// debug
	//outFile.close();

	_averageHeight = sumHeights / (float)_vertices->size();

	computeNormals();
	_normalMap = createNormalMap();
	//computeMaxDelta();
}

OceanTile::OceanTile( const OceanTile& tile, 
							 unsigned int resolution, 
							 const float spacing ):
	_resolution	( resolution ),
	_rowLength	( _resolution + 1 ),
	_numVertices( _rowLength*_rowLength ),
	_vertices	( new osg::Vec3Array ),
	_normals		( new osg::Vec3Array(_numVertices) ),
	_spacing		( spacing ),
	_maxDelta	( 0.f )
{
	unsigned int parentRes = tile.getResolution();
	unsigned int inc = parentRes/_resolution;
	unsigned int inc2 = inc/2;

	_vertices->resize( _numVertices );

	// Take an average of four points
	for (unsigned int y = 0; y < parentRes; y+=inc) 
	{
		for (unsigned int x = 0; x < parentRes; x+=inc) 
		{
			const osg::Vec3f& a = tile.getVertex( x,		 y   );
			const osg::Vec3f& b = tile.getVertex( x+inc2, y   );
			const osg::Vec3f& c = tile.getVertex( x,		 y+inc2 );
			const osg::Vec3f& d = tile.getVertex( x+inc2, y+inc2 );

			osg::Vec3f sum = a + b + c + d;

			(*_vertices)[ array_pos(x/inc, y/inc, _rowLength) ] = sum * 0.25f;
		}
	}

	for( unsigned int i = 0; i < _rowLength-1; ++i )
	{
		// Copy top row into skirt
		(*_vertices)[ array_pos( i, _rowLength-1, _rowLength) ] = (*_vertices)[ i ];

		// Copy first column into skirt
		(*_vertices)[ array_pos( _rowLength-1, i, _rowLength) ] = (*_vertices)[ i*_rowLength ];		
	}

	// Copy corner value
	(*_vertices)[ array_pos( _rowLength-1, _rowLength-1, _rowLength ) ] = (*_vertices)[0];
	
	computeNormals();
}

OceanTile::OceanTile( const OceanTile& copy ):
	_vertices		( copy._vertices ),
	_normals			( copy._normals ),
	_normalMap		( copy._normalMap ),
	_resolution		( copy._resolution ),
	_rowLength		( copy._rowLength ),
	_numVertices	( copy._numVertices ),
	_spacing			( copy._spacing ),
	_maxDelta		( copy._maxDelta ),
	_averageHeight	( copy._averageHeight )
{

}

OceanTile::~OceanTile( void )
{

}

OceanTile& OceanTile::operator=(const OceanTile& rhs) 
{
	if (this != &rhs) 
	{
		_vertices		= rhs._vertices;
		_normals			= rhs._normals;
		_normalMap		= rhs._normalMap;
		_resolution		= rhs._resolution;
		_rowLength		= rhs._rowLength;
		_numVertices	= rhs._numVertices;
		_spacing			= rhs._spacing;
		_maxDelta		= rhs._maxDelta;
		_averageHeight = rhs._averageHeight;
	}
	return *this;
}

void OceanTile::computeNormals( void )
{
	int x1,x2,y1,y2;
	osg::Vec3f a,b,c,d,v1,v2,v3,n1,n2;

	const osg::Vec3f s2 = osg::Vec3f( 0.f,		  -_spacing, 0.f );
	const osg::Vec3f s3 = osg::Vec3f( _spacing, 0.f,		 0.f ); 
	const osg::Vec3f s4 = osg::Vec3f( _spacing, -_spacing, 0.f );

	// Compute normals for an N+2 x N+2 grid to ensure continuous
	// normals for tiles of the same resolution
	// Using first row as last row and first column as last column.

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array( (_resolution+2)*(_resolution+2) );

	for( int y = -1; y < (int)_resolution; ++y )
	{	
		y1 = y % _resolution;
		y2 = (y+1) % _resolution;

		for( int x = -1; x < (int)_resolution; ++x )
		{
			x1 = x % _resolution;
			x2 = (x+1) % _resolution;

			a =		getVertex(x1,		y1);		//	  a|   /|c
			b = s2 + getVertex(x1,		y2);		//		|  / |
			c = s3 + getVertex(x2,		y1);		//		| /  |
			d = s4 + getVertex(x2,		y2);		//	  b|/   |d

			v1 = b - a;
			v2 = b - c;
			v3 = b - d;

			n1 = v2 ^ v1;
			n2 = v3 ^ v2;

			(*normals)[ array_pos(x+1, y+1, _resolution+2) ] += n1;		// a|  /c
			(*normals)[ array_pos(x+1, y+2, _resolution+2) ] += n1;		//  | /
			(*normals)[ array_pos(x+2, y+1, _resolution+2) ] += n1;		// b|/

			(*normals)[ array_pos(x+1, y+2, _resolution+2) ] += n2;		//    /|c
			(*normals)[ array_pos(x+2, y+1, _resolution+2) ] += n2;		//   / |
			(*normals)[ array_pos(x+2, y+2, _resolution+2) ] += n2;		// b/__|d
		}
	}

	for( osg::Vec3Array::iterator itr = normals->begin(); 
		  itr != normals->end(); 
		  ++itr )
	{
		itr->normalize();
	}

	// copy normals into member normal array discarding first row and column;

	unsigned int ptr = 0;

	for(unsigned int y = 1; y <= _resolution+1; ++y )
	{
		for(unsigned int x = 1; x <= _resolution+1; ++x )
		{
			(*_normals)[ptr] = (*normals)[ array_pos(x,y,_resolution+2) ];
			++ptr;
		}
	}
}

void OceanTile::computeMaxDelta( void )
{
	float deltaMax = 0;

	int step = 2;
	int numLevels = 6;

	for (int level=1; level < numLevels; ++level)
	{
		for( unsigned int i=0; i < _resolution; ++i) 
		{
			int posY = i/step * step;
			
			for( unsigned int j=0; j < _resolution; ++j) 
			{
				if (i%step != 0 || j%step != 0) 
				{
					int posX = j/step * step;

					float delta = biLinearInterp(posX, posX+step, posY, posY+step, j, i);
					delta -= getVertex(j, i).z();
					delta = abs(delta);
					deltaMax = std::max(deltaMax, delta);
				}
			}
		}
		step *= 2;
	}
}

float OceanTile::biLinearInterp(int lx, int hx, int ly, int hy, int tx, int ty ) 
{
	float s00 = getVertex(lx, ly).z();
	float s01 = getVertex(hx, ly).z();
	float s10 = getVertex(lx, hy).z();
	float s11 = getVertex(hx, hy).z();

	int dx = hx - lx;
	int dtx = tx - lx;
	float v0 = (s01 - s00)/dx*dtx + s00;
	float v1 = (s11 - s10)/dx*dtx + s10;
	float value = (v1 - v0)/(hy - ly)*(ty - ly) + v0;

	return value;
}

osg::ref_ptr<osg::Texture2D> OceanTile::createNormalMap( void ) const 
{
	if( !_normals.valid() )
		return NULL;

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;

	unsigned char* pixels = new unsigned char[_resolution*_resolution*3];	

	unsigned int idx = 0;
	unsigned int i = 0;

	for(unsigned int r = 0; r < _resolution; ++r )
	{
		for(unsigned int c = 0; c < _resolution; ++c )
		{
			idx = i*3;

			pixels[idx]   = (unsigned char)(127.f * getNormal(c,r).x() + 128.f);
			pixels[idx+1] = (unsigned char)(127.f * getNormal(c,r).y() + 128.f);
			pixels[idx+2] = (unsigned char)(127.f * getNormal(c,r).z() + 128.f);

			i++;
		}
	}

	osg::Image* img = new osg::Image;
	img->setImage(_resolution, _resolution, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, pixels, osg::Image::USE_NEW_DELETE, 1);

	// debug
	//static int count = 0;
	//std::stringstream ss;
	//ss << "Tile_" << count << ".bmp";
	//osgDB::writeImageFile( *img, ss.str() );
	//++count;

	texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
	texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
	texture->setWrap	( osg::Texture::WRAP_S,		 osg::Texture::REPEAT );
	texture->setWrap	( osg::Texture::WRAP_T,		 osg::Texture::REPEAT );
	texture->setImage( img );

	return texture;
}

