#include <osgOcean/FFTOceanSurface>

using namespace osgOcean;

FFTOceanSurface::FFTOceanSurface( unsigned int FFTGridSize,
									 unsigned int resolution,
									 unsigned int numTiles, 
									 const osg::Vec2f& windDirection,
									 float windSpeed,
									 float depth,
									 bool isChoppy,
									 float choppyFactor,
									 float animLoopTime,
									 unsigned int numFrames ):

	_tileSize		 ( FFTGridSize ),	
	_tileResolution ( resolution ),
	_numTiles		 ( numTiles ),
	_totalPoints	 ( _tileSize * _numTiles + 1 ),
	_pointSpacing	 ( _tileResolution / _tileSize ),
	_windDirection	 ( windDirection ),
	_windSpeed		 ( windSpeed ),
	_waveConstant	 ( double(_tileSize)*(1e-8) ),
	_depth			 ( depth ),
	_cycleTime		 ( animLoopTime ),
	_choppyFactor	 ( choppyFactor ),
	_isChoppy		 ( isChoppy ),
	_oldFrame		 ( 0 ),
	_numVertices	 ( 0 ),
	_newNumVertices ( 0 ),
	_numLevels		 ( (unsigned int) ( log( (float)_tileSize) / log(2.f) ) ),
	_startPos		 ( -float( (_tileResolution+1)*_numTiles) * 0.5f, float( (_tileResolution+1)*_numTiles) * 0.5f ),
	_THRESHOLD		 ( 3.f ),
	_VRES				 ( 1024 ),
	_NUMFRAMES		 ( numFrames ),
	_activeVertices ( new osg::Vec3Array ),
	_activeNormals	 ( new osg::Vec3Array ),
	_waveTopColor	 ( 0.180392f, 0.329411f, 0.470588f ),
	_waveBottomColor( 0.039215f, 0.164705f, 0.223529f ),
	_useCrestFoam	 ( false ),
	_foamHeight		 ( 2.2 ),
	_isStateDirty	 ( true ),
	_averageHeight	 ( 0.f ),
	_lightColor		 ( 0.411764705f, 0.54117647f, 0.6823529f, 1.f )
{
	_stateset = new osg::StateSet;

	setUserData( new OceanDataType(*this, _NUMFRAMES, 25) );
	addCullCallback( new OceanAnimationCallback );
	addUpdateCallback( new OceanAnimationCallback );
}

FFTOceanSurface::FFTOceanSurface( const FFTOceanSurface& copy, const osg::CopyOp& copyop ):
	OceanTechnique	 ( copy, copyop ),
	_tileSize		 ( copy._tileSize ),
	_tileResolution ( copy._tileResolution ),
	_numTiles		 ( copy._numTiles ),
	_totalPoints	 ( copy._totalPoints ),
	_pointSpacing	 ( copy._pointSpacing ),
	_windDirection	 ( copy._windDirection ),
	_windSpeed		 ( copy._windSpeed ),
	_waveConstant	 ( copy._waveConstant ),
	_depth			 ( copy._depth ),
	_cycleTime		 ( copy._cycleTime ),
	_choppyFactor	 ( copy._choppyFactor ),
	_isChoppy		 ( copy._isChoppy ),
	_oldFrame		 ( copy._oldFrame ),
	_numVertices	 ( copy._numVertices ),
	_newNumVertices ( copy._newNumVertices ),
	_numLevels		 ( copy._numLevels ),
	_startPos		 ( copy._startPos ),
	_THRESHOLD		 ( copy._THRESHOLD ),
	_VRES				 ( copy._VRES ),
	_NUMFRAMES		 ( copy._NUMFRAMES ),
	_minDist			 ( copy._minDist ),
	_mipmapData		 ( copy._mipmapData ),
	_oceanGeom		 ( copy._oceanGeom ),
	_activeVertices ( copy._activeVertices ),
	_activeNormals	 ( copy._activeNormals ),
	_waveTopColor	 ( copy._waveTopColor ),
	_waveBottomColor( copy._waveBottomColor ),
	_useCrestFoam	 ( copy._useCrestFoam ),
	_foamHeight		 ( copy._foamHeight ),
	_isStateDirty	 ( copy._isStateDirty ),
	_averageHeight	 ( copy._averageHeight ),
	_lightColor		 ( copy._lightColor )
{

}

FFTOceanSurface::~FFTOceanSurface(void)
{
}

void FFTOceanSurface::build( void )
{
	computeSea( _NUMFRAMES );
	createOceanTiles();
	computeVertices(0);
	computePrimitives();

	initStateSet();

	_isDirty =  false;
	_isStateDirty = false;
}

void FFTOceanSurface::initStateSet( void )
{
	_stateset=new osg::StateSet;

	// Environment map	
	_stateset->addUniform( new osg::Uniform("uEnableGlobalReflections", true ) );
	_stateset->addUniform( new osg::Uniform("uEnvironmentMap", ENV_MAP ) );
	_stateset->setTextureAttributeAndModes( ENV_MAP, _environmentMap.get(), osg::StateAttribute::ON );
	
	// Foam
	_stateset->addUniform( new osg::Uniform("uEnableCrestFoam", _useCrestFoam ) );
	_stateset->addUniform( new osg::Uniform("uFoamCapHeight",  _foamHeight ) );
	_stateset->addUniform( new osg::Uniform("uFoamMap",		  FOAM_MAP ) );

	if( _useCrestFoam )
	{
		osg::Texture2D* foam_tex = createTexture( "resources/textures/sea_foam.png", osg::Texture::REPEAT );
		_stateset->setTextureAttributeAndModes( FOAM_MAP, foam_tex, osg::StateAttribute::ON );
	}

	// Noise
	_stateset->addUniform( new osg::Uniform("uNoiseMap",		 NORMAL_MAP ) );
	_stateset->addUniform( new osg::Uniform("uNoiseCoords0",	 computeNoiseCoords(  2, 5, 2, 8, 32, 256, 0.f ) ) );
	_stateset->addUniform( new osg::Uniform("uNoiseCoords1",	 computeNoiseCoords( -4, 3, 1, 32, 8,  256, 0.f ) ) );

	_stateset->setTextureAttributeAndModes( NORMAL_MAP, _mipmapData[0][0].getNormalMap().get(), osg::StateAttribute::ON );

	// Colouring
	_waveTopColor = osg::Vec3f(0.192156862, 0.32549019, 0.36862745098 );
	_waveBottomColor = osg::Vec3f(0.11372549019, 0.219607843, 0.3568627450 );

	osg::Vec4f waveTop = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveTopColor,1.f) );
	osg::Vec4f waveBot = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveBottomColor,1.f) );

	_stateset->addUniform( new osg::Uniform("uWaveTop",	 waveTop ) );
	_stateset->addUniform( new osg::Uniform("uWaveBot",	 waveBot ) );
		
	_stateset->addUniform( new osg::Uniform("uEyePosition",	 osg::Vec3f() ) );
	
	osg::Shader* vShader = new osg::Shader( osg::Shader::VERTEX );
	vShader->setName("ocean_vertex_shader");
	osg::Shader* fShader = new osg::Shader( osg::Shader::FRAGMENT );
	fShader->setName("ocean_fragment_shader");

	if( !vShader->loadShaderSourceFromFile( "resources/shaders/water.v" ) )
		osg::notify(osg::WARN) << "ERROR: Could not load vertex shader source!" << std::endl;

	if( !fShader->loadShaderSourceFromFile( "resources/shaders/water.f" ) )
		osg::notify(osg::WARN) << "ERROR: Could not load fragment shader source!" << std::endl;

	osg::Program* program = new osg::Program;
	program->addShader( vShader );
	program->addShader( fShader );

	_stateset->setAttributeAndModes( program, osg::StateAttribute::ON );

	_isStateDirty = false;
}

void FFTOceanSurface::computeSea( unsigned int totalFrames )
{
	FFTSimulation FFTSim( _tileSize, _windDirection, _windSpeed, _waveConstant, _tileResolution, _cycleTime );

	_mipmapData.resize( totalFrames );

	_averageHeight = 0.f;

	for( unsigned int frame = 0; frame < totalFrames; ++frame )
	{
		osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;
		osg::ref_ptr<osg::Vec2Array> displacements = new osg::Vec2Array;

		float time = _cycleTime * ( float(frame) / float(totalFrames) );

		FFTSim.set_time( time );
		FFTSim.compute_heights( heights.get() );

		if(_isChoppy)
			FFTSim.compute_displacements( _choppyFactor, displacements.get() );

		_mipmapData[frame].resize( _numLevels );

		// Level 0
		if(_isChoppy)
			_mipmapData[frame][0] = OceanTile( heights.get(), displacements.get(), _tileSize, _pointSpacing );
		else
			_mipmapData[frame][0] = OceanTile( heights.get(), _tileSize, _pointSpacing );

		_averageHeight = _mipmapData[frame][0].getAverageHeight();

		// Levels 1 -> Max Level
		for(unsigned int level = 1; level < _numLevels; ++level )
		{
			OceanTile& lastTile = _mipmapData[frame][level-1];

			_mipmapData[frame][level] = OceanTile( lastTile, _tileSize >> level, _tileSize/(_tileSize>>level)*_pointSpacing );
		}
	}

	_averageHeight /= (float)totalFrames;
}

void FFTOceanSurface::createOceanTiles( void )
{
	MipmapGeometry::BORDER_TYPE border = MipmapGeometry::BORDER_NONE;

	_numVertices = 0;

	_oceanGeom.resize( _numTiles );

	osg::ref_ptr<osg::Vec4Array> colours = new osg::Vec4Array;
	colours->push_back( osg::Vec4f(1.f, 1.f,1.f,1.f) );

	for(int y = 0; y < (int)_numTiles; ++y )
	{
		for(int x = 0; x < (int)_numTiles; ++x )
		{
			if(x == _numTiles-1 && y == _numTiles-1)
				border = MipmapGeometry::BORDER_XY;
			else if(x == _numTiles-1)		
				border = MipmapGeometry::BORDER_X;
			else if(y==_numTiles-1)
				border = MipmapGeometry::BORDER_Y;
			else 
				border = MipmapGeometry::BORDER_NONE;
			
			MipmapGeometry* patch = new MipmapGeometry( _numLevels-1, _numLevels, 0, border );

			patch->setUseDisplayList( false );
			patch->setVertexArray( _activeVertices.get() );
			patch->setNormalArray( _activeNormals.get() );
			patch->setColorArray	( colours.get() );
			patch->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
			patch->setColorBinding( osg::Geometry::BIND_OVERALL );
			patch->setDataVariance( osg::Object::DYNAMIC );
			patch->setIdx( _numVertices );

			addDrawable( patch );

			_oceanGeom[y].push_back( patch );

			unsigned int verts = 0;
			unsigned int s = 2 << (_numLevels-_numLevels);

			verts = s * s;

			if(x == _numTiles-1 )							// If on right border add extra column
				verts += s;
			if(y == _numTiles-1 )							// If on bottom border add extra row
				verts += s;
			if(x == _numTiles-1 && y == _numTiles-1)	// If corner piece add corner vertex
				verts += 1;

			_numVertices += verts;
		}
	}

	_activeVertices->resize( _numVertices );
	_activeNormals->resize( _numVertices );

// Correct dMin calculations for geomipmap distances. Not used at the moment
//	float T = (2.0f * TRESHOLD) / VRES;
//	float A = 1.0f / (float)tan(FOV / 2.0f);
//	float C = A / T;

	for(unsigned int d = 0; d < _numLevels; ++d)
	{
		_minDist.push_back( d * (_tileResolution) +_tileResolution/2);
		_minDist.back() *= _minDist.back();
	}
}

void FFTOceanSurface::computeVertices( unsigned int frame )
{
	// Only resize vertex/normal arrays if more are needed
	if(_newNumVertices > _numVertices )
	{
		_numVertices = _newNumVertices;
		_activeVertices->resize(_numVertices);
		_activeNormals->resize(_numVertices);
	}

	osg::Vec3f tileOffset,vertexOffset,vertex;
	
	tileOffset.x() = -int((_tileResolution+1)*_numTiles) / 2;
	tileOffset.y() =  int((_tileResolution+1)*_numTiles) / 2;

	unsigned int ptr = 0;

	const std::vector<OceanTile>& curData = _mipmapData[frame];

	for(unsigned int y = 0; y < _numTiles; ++y )
	{	
		tileOffset.y() = _startPos.y() - y*_tileResolution;

		for(unsigned int x = 0; x < _numTiles; ++x )
		{
			tileOffset.x() = _startPos.x() + x*_tileResolution;

			MipmapGeometry* tile = getTile(x,y);
			const OceanTile& data = curData[ tile->getLevel() ];

			for(unsigned int row = 0; row < tile->getColLen(); ++row )
			{
				vertexOffset.y() = data.getSpacing()*-float(row) + tileOffset.y();

				for(unsigned int col = 0; col < tile->getRowLen(); ++col )
				{
					vertexOffset.x() = data.getSpacing()*float(col) + tileOffset.x();

					(*_activeVertices)[ptr] = data.getVertex(col,row) + vertexOffset;
					(*_activeNormals) [ptr] = data.getNormal(col,row);
					++ptr;
				}
			}
		}
	}
}

void FFTOceanSurface::update( unsigned int frame, const osg::Vec3f& eye )
{
	if(_isDirty)
		build();
	else if(_isStateDirty)
		initStateSet();

	static double time = 0.0;

	getStateSet()->getUniform("uEyePosition")->set(eye);
	getStateSet()->getUniform("uNoiseCoords0")->set( computeNoiseCoords(  2, 5, 2, 8, 32, 256, time ) );
	getStateSet()->getUniform("uNoiseCoords1")->set( computeNoiseCoords( -4, 3, 1, 32, 8, 256, time ) );

	time+=0.009;

	if( updateMipmaps( eye, frame ) )
	{
		computeVertices( frame );
		computePrimitives();
		getStateSet()->setTextureAttributeAndModes( NORMAL_MAP, _mipmapData[frame][0].getNormalMap().get() );
	}
	else if( frame != _oldFrame )
	{
		computeVertices( frame );
		getStateSet()->setTextureAttributeAndModes( NORMAL_MAP, _mipmapData[frame][0].getNormalMap().get() );
	}

	_oldFrame = frame;
}

bool FFTOceanSurface::updateMipmaps( const osg::Vec3f& eye, unsigned int frame )
{
	static unsigned int count = 0;

	bool updated = false;

	_newNumVertices = 0;

	for( unsigned int y = 0; y < _numTiles; ++y)
	{
		for( unsigned int x = 0; x < _numTiles; ++x)
		{
			osg::Vec3f distanceToTile = getTile(x,y)->getBound().center() - eye;
			
			unsigned int mipmapLevel = 0;

			for( unsigned int m = 0; m < _minDist.size(); ++m )
			{
				if( distanceToTile.length2() > _minDist.at(m) )
					mipmapLevel = m;
			}

			if( getTile(x,y)->getLevel() != mipmapLevel )
				updated = true;

			getTile(x,y)->setLevel( mipmapLevel );
			getTile(x,y)->setIdx( _newNumVertices );
			
			unsigned int verts = 0;
			unsigned int size = getTile(x,y)->getResolution();

			verts = size * size;

			if(x == _numTiles-1 )
				verts += size;
			if(y == _numTiles-1 )
				verts += size;
			if(x == _numTiles-1 && y == _numTiles-1)
				verts += 1;

			_newNumVertices += verts;
		}
	}

	return updated;	
}

void FFTOceanSurface::computePrimitives( void )
{
	int x1 = 0;
	int y1 = 0;
	int size = 0;

	for(unsigned int y = 0; y < _numTiles; ++y)
	{
		for(unsigned int x = 0; x < _numTiles; ++x )
		{
			x+1 > _numTiles-1 ? x1 = _numTiles-1 : x1 = x+1;
			y+1 > _numTiles-1 ? y1 = _numTiles-1 : y1 = y+1;

			MipmapGeometry* cTile  = getTile(x,y);		// Current tile
			MipmapGeometry* xTile  = getTile(x1,y);	// Right Tile
			MipmapGeometry* yTile  = getTile(x,y1);	// Bottom Tile
			MipmapGeometry* xyTile = getTile(x1,y1);	// Bottom right Tile

			// First clear old primitive sets
			cTile->removePrimitiveSet(0, cTile->getNumPrimitiveSets() );

			addMainBody(cTile);

			if( x < _numTiles-1 )
				addRightBorder( cTile, xTile );

			if( y < _numTiles-1 )
				addBottomBorder( cTile, yTile );

			addCornerPatch( cTile, xTile, yTile, xyTile );
		}
	}
}

void FFTOceanSurface::addMainBody( MipmapGeometry* cTile )
{
	unsigned int degenX = cTile->getRowLen()-1;
	unsigned int degenY = cTile->getColLen()-1;

	unsigned int numDegens = (cTile->getColLen()-1)*2-2;
	unsigned int stripSize = (cTile->getRowLen()*2)*(cTile->getColLen()-1) + numDegens;
	unsigned int i = 0;

	// Generate 1 tristrip using degen triangles
	osg::DrawElementsUInt* strip = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, stripSize );

	for( unsigned int row = 0; row < cTile->getColLen()-1; ++row )
	{
		for( unsigned int col = 0; col < cTile->getRowLen(); ++col )
		{
			(*strip)[i]   = cTile->getIndex( col,	 row   );
			(*strip)[i+1] = cTile->getIndex( col,	 row+1 );
			i+=2;

			if( col == degenX && row+1 != degenY )
			{
				(*strip)[i]   = cTile->getIndex( col, row+1 );
				(*strip)[i+1] = cTile->getIndex( 0,	  row+1 );
				i+=2;
			}
		}
	}
	cTile->addPrimitiveSet( strip );
}

void FFTOceanSurface::addRightBorder( MipmapGeometry* cTile, MipmapGeometry* xTile )
{
	unsigned int endCol = cTile->getRowLen() - 1;

	// Same level to the right
	if( cTile->getLevel() == xTile->getLevel() )
	{
		//	3	2
		//
		//	0	1

		for(unsigned int r = 0; r < cTile->getColLen()-1; ++r)	
		{
			osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 4);

			(*fan)[0] = cTile->getIndex( endCol,  r+1 );		
			(*fan)[1] = xTile->getIndex( 0,		  r+1 );		
			(*fan)[2] = xTile->getIndex( 0,		  r   );		
			(*fan)[3] = cTile->getIndex( endCol,  r   );		

			cTile->addPrimitiveSet( fan );
		}
	}
	// low res to the right
	else if( cTile->getLevel() < xTile->getLevel() )
	{
		unsigned int diff = cTile->getResolution() / xTile->getResolution(); 
		unsigned int cPts = diff + 1;		
		unsigned int start = 0;

		//	1	0
		//	2
		//	3	4
		
		for(unsigned int r = 0; r < xTile->getColLen()-1; ++r )
		{
			osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
			fan->reserve( cPts+2 );	

			fan->push_back( xTile->getIndex( 0, r ) );

			start = r*diff;

			for(unsigned int i = 0; i < cPts; ++i)
			{
				fan->push_back( cTile->getIndex( endCol, start+i ) );
			}

			fan->push_back( xTile->getIndex( 0, r+1 ) );

			cTile->addPrimitiveSet( fan );
		}
	}
	// high res to the right
	else if( cTile->getLevel() > xTile->getLevel() )
	{
		unsigned int diff = xTile->getResolution() / cTile->getResolution(); 
		unsigned int xPts = diff + 1;	
		unsigned int start = 0;

		//	4		3
		//			2
		//	0		1

		for(unsigned int r = 0; r < cTile->getColLen()-1; ++r )
		{
			osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
			fan->reserve( xPts+2 );	

			fan->push_back( cTile->getIndex( endCol, r+1 ) );

			start = (r+1)*diff;

			for(unsigned int i = 0; i < xPts; ++i )
			{
				fan->push_back( xTile->getIndex( 0, start-i ) );
			}

			fan->push_back( cTile->getIndex( endCol, r ) );

			cTile->addPrimitiveSet( fan );
		}
	}
}

void FFTOceanSurface::addBottomBorder( MipmapGeometry* cTile, MipmapGeometry* yTile )
{
	unsigned int endRow = cTile->getColLen() - 1;

	// Same res below
	if( cTile->getLevel() == yTile->getLevel() )
	{
		unsigned int i = 0; 

		osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, cTile->getRowLen()*2);

		for(unsigned int c = 0; c < cTile->getRowLen(); ++c)
		{
			(*fan)[i]   = cTile->getIndex( c, endRow	);	//	0		2
			(*fan)[i+1] = yTile->getIndex( c, 0			);	// 1		3
			i+=2;
		}

		cTile->addPrimitiveSet( fan );
	}
	// lower res below
	else if( cTile->getLevel() < yTile->getLevel() )
	{
		unsigned int diff = cTile->getResolution() / yTile->getResolution(); 
		unsigned int cPts = diff + 1;
		unsigned int start = 0;

		// 4	3	2
		//
		// 0		1

		for(unsigned int c = 0; c < yTile->getRowLen()-1; ++c)
		{
			osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
			fan->reserve( cPts+2 );

			fan->push_back( yTile->getIndex( c,   0 ) );
			fan->push_back( yTile->getIndex( c+1, 0 ) );

			start = (c+1)*diff;

			for( unsigned int i = 0; i < cPts; ++i )
			{
				fan->push_back( cTile->getIndex( start-i, endRow ) );
			}

			cTile->addPrimitiveSet( fan );
		}
	}
	// Higher res below
	else if( cTile->getLevel() > yTile->getLevel() )
	{
		unsigned int diff = yTile->getResolution() / cTile->getResolution(); 
		unsigned int yPts = diff + 1;		
		unsigned int start = 0;

		//	1	 	0
		//
		// 2	3	4

		for(unsigned int c = 0; c < cTile->getRowLen()-1; ++c)
		{
			osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
			fan->reserve( yPts+2 );

			fan->push_back( cTile->getIndex( c+1, endRow ) );
			fan->push_back( cTile->getIndex( c,   endRow ) );

			start = c*diff;

			for( unsigned int i = 0; i < yPts; ++i )
			{
				fan->push_back( yTile->getIndex( start+i, 0 ) );
			}

			cTile->addPrimitiveSet( fan );
		}
	}
}

void FFTOceanSurface::addCornerPatch( MipmapGeometry* cTile, MipmapGeometry* xTile, MipmapGeometry* yTile, MipmapGeometry* xyTile )
{
	// CORNER PATCH
	// ------------

	int x_points = xTile->getResolution() / cTile->getResolution();
	int y_points = yTile->getResolution() / cTile->getResolution(); 

	unsigned int curSize   = cTile->getResolution()-1;
	unsigned int botSize	  = yTile->getResolution()-1;
	unsigned int rightSize = xTile->getResolution()-1;

	if( cTile->getBorder() == MipmapGeometry::BORDER_NONE )
	{
		// Low res bottom
		if( y_points == 0 )
		{
			// Low res right
			if( x_points == 0 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

				(*fan)[0] = cTile->getIndex ( curSize,		curSize	 ); //		5	4
				(*fan)[1] = cTile->getIndex ( curSize-1,	curSize	 ); //	
				(*fan)[2] = yTile->getIndex ( botSize,		0			 ); // 1	0	
				(*fan)[3] = xyTile->getIndex( 0,				0			 ); // 
				(*fan)[4] = xTile->getIndex ( 0,				rightSize ); //	2		3
				(*fan)[5] = cTile->getIndex ( curSize,		curSize-1 );	

				cTile->addPrimitiveSet( fan );
			}
			// same res right
			else if( x_points == 1 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 5);

				(*fan)[0] = yTile->getIndex ( botSize,		0				);	//
				(*fan)[1] = xyTile->getIndex( 0,				0				);	//	4	3	2
				(*fan)[2] = xTile->getIndex ( 0,				rightSize	);	//
				(*fan)[3] = cTile->getIndex ( curSize,		curSize		);	//	0		1
				(*fan)[4] = cTile->getIndex ( curSize-1,	curSize		);	//

				cTile->addPrimitiveSet( fan );
			}
			// high res right
			else if( x_points == 2 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

				(*fan)[0] = yTile->getIndex	( botSize,		0				);	//	5	4	3
				(*fan)[1] = xyTile->getIndex	( 0,				0				);	//
				(*fan)[2] = xTile->getIndex	( 0,				rightSize	);	//			2
				(*fan)[3] = xTile->getIndex	( 0,				rightSize-1	);	//	
				(*fan)[4] = cTile->getIndex	( curSize,		curSize		);	//	0		1
				(*fan)[5] = cTile->getIndex	( curSize-1,	curSize		);	

				cTile->addPrimitiveSet( fan );
			}
		}
		// same res bottom
		else if( y_points == 1 )
		{
			// Low res right
			if( x_points == 0 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 7);

				(*fan)[0] = cTile->getIndex ( curSize,		curSize	 ); //			6	5
				(*fan)[1] = cTile->getIndex ( curSize-1,	curSize	 ); //	
				(*fan)[2] = yTile->getIndex ( botSize-1,	0			 ); // 1		0	
				(*fan)[3] = yTile->getIndex ( botSize,		0			 ); // 
				(*fan)[4] = xyTile->getIndex( 0,				0			 ); //		2	3	4
				(*fan)[5] = xTile->getIndex ( 0,				rightSize );
				(*fan)[6] = cTile->getIndex ( curSize,		curSize-1 );	

				cTile->addPrimitiveSet( fan );
			}
			// same res right
			else if( x_points == 1 )
			{
				osg::DrawElementsUInt* strip = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 6);

				(*strip)[0] = cTile->getIndex ( curSize-1, curSize	  ); //	0	2	4
				(*strip)[1] = yTile->getIndex ( botSize-1, 0			  ); //	
				(*strip)[2] = cTile->getIndex ( curSize,	 curSize	  ); // 1	3	5
				(*strip)[3] = yTile->getIndex ( botSize,	 0			  ); // 
				(*strip)[4] = xTile->getIndex ( 0,			 rightSize );	
				(*strip)[5] = xyTile->getIndex( 0,			 0			  );

				cTile->addPrimitiveSet( strip );
			}
			// high res right
			else if( x_points == 2 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 7);

				(*fan)[0] = cTile->getIndex ( curSize,		curSize		); //	1	0	6
				(*fan)[1] = cTile->getIndex ( curSize-1,	curSize		); //	
				(*fan)[2] = yTile->getIndex ( botSize-1,	0				); //			5
				(*fan)[3] = yTile->getIndex ( botSize,		0				); //
				(*fan)[4] = xyTile->getIndex( 0,				0				); // 2	3	4
				(*fan)[5] = xTile->getIndex ( 0,				rightSize	);
				(*fan)[6] = xTile->getIndex ( 0,				rightSize-1 );

				cTile->addPrimitiveSet( fan );
			}
		}
		// high res bottom
		else if( y_points == 2 )
		{
			// Low res right
			if( x_points == 0 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

				(*fan)[0] = xTile->getIndex( 0,			 rightSize	);	//	1		0
				(*fan)[1] = cTile->getIndex( curSize,	 curSize-1	);	//	
				(*fan)[2] = cTile->getIndex( curSize,	 curSize		);	//	2			
				(*fan)[3] = yTile->getIndex( botSize-1, 0				);	// 
				(*fan)[4] = yTile->getIndex( botSize,	 0				);	// 3	4	5
				(*fan)[5] = xyTile->getIndex( 0,			 0				);					

				cTile->addPrimitiveSet( fan );
			}
			// same res right
			if( x_points == 1 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 5);

				(*fan)[0] = xTile->getIndex ( 0,			  rightSize	);	//	1		0
				(*fan)[1] = cTile->getIndex ( curSize,	  curSize	);	//	
				(*fan)[2] = yTile->getIndex ( botSize-1, 0			);	// 
				(*fan)[3] = yTile->getIndex ( botSize,	  0			);	// 2	3	4
				(*fan)[4] = xyTile->getIndex( 0,			  0			);					

				cTile->addPrimitiveSet( fan );
			}
			// high res right
			if( x_points == 2 )
			{
				osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

				(*fan)[0] = cTile->getIndex ( curSize,		curSize		);	//	
				(*fan)[1] = yTile->getIndex ( botSize-1,	0				);	// 0		5
				(*fan)[2] = yTile->getIndex ( botSize,		0				);	// 
				(*fan)[3] = xyTile->getIndex( 0,				0				);	//			4
				(*fan)[4] = xTile->getIndex ( 0,				rightSize	);	//
				(*fan)[5] = xTile->getIndex ( 0,				rightSize-1	);	//	1	2	3			

				cTile->addPrimitiveSet( fan );
			}
		}
	}
}

osg::Vec3f FFTOceanSurface::computeNoiseCoords(int a, int b, double v, float reps, float reps2, int tileLength, float time)
{
	double s = sqrt( float(a * a + b * b) );

	double t = double(tileLength) / reps * s / v;

	osg::Vec2f d(a, b);

	d = d * ( v / s / (tileLength/reps) );

	osg::Vec2f noisePos = osg::Vec2f() + d * fmod( double(time), t );
	
	return osg::Vec3f( noisePos, (1.f/float(tileLength)) * reps2 );
}

float FFTOceanSurface::getSurfaceHeight( void )
{
	return _averageHeight;
}

osg::Texture2D* FFTOceanSurface::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
	osg::Texture2D* tex = new osg::Texture2D();

	//tex->setFilter	(osg::Texture::MIN_FILTER,	osg::Texture::LINEAR_MIPMAP_LINEAR);
	tex->setFilter	(osg::Texture::MIN_FILTER,	osg::Texture::LINEAR);
	tex->setFilter	(osg::Texture::MAG_FILTER,	osg::Texture::LINEAR);
	tex->setWrap	(osg::Texture::WRAP_S,		wrap);
	tex->setWrap	(osg::Texture::WRAP_T,		wrap);
	tex->setImage	(osgDB::readImageFile(name.c_str()));

	return tex;
}

// -------------------------------
//     Callback implementation
// -------------------------------

FFTOceanSurface::OceanDataType::OceanDataType(	FFTOceanSurface& ocean, unsigned int numFrames, unsigned int fps ):
	_oceanSurface	( ocean ),
	_NUMFRAMES		( numFrames ),
	_time				( 0.f ),
	_FPS				( fps ), 
	_msPerFrame		( 1000.f/(float)fps ),
	_frame			( 0 ),
	_oldTime			( 0 ),
	_newTime			( 0 )
{}

FFTOceanSurface::OceanDataType::OceanDataType( const OceanDataType& copy, const osg::CopyOp& copyop ):
	_oceanSurface	( copy._oceanSurface ),
	_NUMFRAMES		( copy._NUMFRAMES ),
	_eye				( copy._eye ),
	_time				( copy._time ),
	_FPS				( copy._FPS ), 
	_msPerFrame		( copy._msPerFrame ),
	_frame			( copy._frame ),
	_oldTime			( copy._oldTime ),
	_newTime			( copy._newTime )
{}

void FFTOceanSurface::OceanDataType::updateOcean( void )
{
	_oldTime = _newTime;
	_newTime = osg::Timer::instance()->tick();

	_time += osg::Timer::instance()->delta_m(_oldTime, _newTime);

	if( _time >= _msPerFrame )
	{
		_frame += ( _time / _msPerFrame );

		if( _frame >= _NUMFRAMES ) 
			_frame = _frame%_NUMFRAMES; 

		_time = fmod( _time, _msPerFrame );
	}

	_oceanSurface.update( _frame, _eye );
}

void FFTOceanSurface::OceanAnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	osg::ref_ptr<OceanDataType> oceanData = dynamic_cast<OceanDataType*> ( node->getUserData() );

	if( oceanData )
	{
		// If cull visitor update the current eye position
		if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
		{
			osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
			oceanData->setEye( cv->getEyePoint() );
		}
		else if( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
			oceanData->updateOcean();
		}
	}

	traverse(node, nv); 
}
