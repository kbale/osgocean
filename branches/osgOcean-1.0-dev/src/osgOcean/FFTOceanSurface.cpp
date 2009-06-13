/*
* This source file is part of the osgOcean library
* 
* Copyright (C) 2009 Kim Bale
* Copyright (C) 2009 The University of Hull, UK
* 
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free Software
* Foundation; either version 3 of the License, or (at your option) any later
* version.

* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
* http://www.gnu.org/copyleft/lesser.txt.
*/

#include <osgOcean/FFTOceanSurface>
#include <osgOcean/ShaderManager>

using namespace osgOcean;

#define USE_LOCAL_SHADERS 1

FFTOceanSurface::FFTOceanSurface( unsigned int FFTGridSize,
                                  unsigned int resolution,
                                  unsigned int numTiles, 
                                  const osg::Vec2f& windDirection,
                                  float windSpeed,
                                  float depth,
                                  float waveScale,
                                  bool isChoppy,
                                  float choppyFactor,
                                  float animLoopTime,
                                  unsigned int numFrames ):

    _tileSize       ( FFTGridSize ),
    _noiseTileSize  ( FFTGridSize ),
    _tileResolution ( resolution ),
    _tileResInv     ( 1.f / float(resolution) ),
    _noiseTileRes   ( resolution ),
    _numTiles       ( numTiles ),
    _totalPoints    ( _tileSize * _numTiles + 1 ),
    _pointSpacing   ( _tileResolution / _tileSize ),
    _windDirection  ( windDirection ),
    _noiseWindDir   ( windDirection ),
    _windSpeed      ( windSpeed ),
    _noiseWindSpeed ( windSpeed ),
    _waveScale      ( waveScale ),
    _noiseWaveScale ( waveScale ),
    _depth          ( depth ),
    _cycleTime      ( animLoopTime ),
    _choppyFactor   ( choppyFactor ),
    _isChoppy       ( isChoppy ),
    _isEndless      ( false ),
    _oldFrame       ( 0 ),
    _numVertices    ( 0 ),
    _newNumVertices ( 0 ),
    _fresnelMul     ( 0.7 ),
    _numLevels      ( (unsigned int) ( log( (float)_tileSize) / log(2.f) )+1),
    _startPos       ( -float( (_tileResolution+1)*_numTiles) * 0.5f, float( (_tileResolution+1)*_numTiles) * 0.5f ),
    _THRESHOLD      ( 3.f ),
    _VRES           ( 1024 ),
    _NUMFRAMES      ( numFrames ),
    _activeVertices ( new osg::Vec3Array ),
    _activeNormals  ( new osg::Vec3Array ),
    _waveTopColor   ( 0.192156862f, 0.32549019f, 0.36862745098f ),
    _waveBottomColor( 0.11372549019f, 0.219607843f, 0.3568627450f ),
    _useCrestFoam   ( false ),
    _foamCapBottom  ( 2.2f ),
    _foamCapTop     ( 3.0f ),
    _isStateDirty   ( true ),
    _averageHeight  ( 0.f ),
    _lightColor     ( 0.411764705f, 0.54117647f, 0.6823529f, 1.f )
{
    addResourcePaths();

    _stateset = new osg::StateSet;

    setUserData( new OceanDataType(*this, _NUMFRAMES, 25) );
    addCullCallback( new OceanAnimationCallback );
    addUpdateCallback( new OceanAnimationCallback );
}

FFTOceanSurface::FFTOceanSurface( const FFTOceanSurface& copy, const osg::CopyOp& copyop ):
    OceanTechnique  ( copy, copyop ),
    _tileSize       ( copy._tileSize ),
    _noiseTileSize  ( copy._noiseTileSize ),
    _tileResolution ( copy._tileResolution ),
    _tileResInv     ( copy._tileResInv ),
    _noiseTileRes   ( copy._noiseTileRes ),
    _numTiles       ( copy._numTiles ),
    _totalPoints    ( copy._totalPoints ),
    _pointSpacing   ( copy._pointSpacing ),
    _windDirection  ( copy._windDirection ),
    _noiseWindDir   ( copy._noiseWindDir ),
    _windSpeed      ( copy._windSpeed ),
    _noiseWindSpeed ( copy._noiseWindSpeed ),
    _waveScale      ( copy._waveScale ),
    _noiseWaveScale ( copy._noiseWaveScale ),
    _depth          ( copy._depth ),
    _cycleTime      ( copy._cycleTime ),
    _choppyFactor   ( copy._choppyFactor ),
    _isChoppy       ( copy._isChoppy ),
    _isEndless      ( copy._isEndless ),
    _oldFrame       ( copy._oldFrame ),
    _numVertices    ( copy._numVertices ),
    _newNumVertices ( copy._newNumVertices ),
    _fresnelMul     ( copy._fresnelMul ),
    _numLevels      ( copy._numLevels ),
    _startPos       ( copy._startPos ),
    _THRESHOLD      ( copy._THRESHOLD ),
    _VRES           ( copy._VRES ),
    _NUMFRAMES      ( copy._NUMFRAMES ),
    _minDist        ( copy._minDist ),
    _mipmapData     ( copy._mipmapData ),
    _oceanGeom      ( copy._oceanGeom ),
    _activeVertices ( copy._activeVertices ),
    _activeNormals  ( copy._activeNormals ),
    _environmentMap ( copy._environmentMap ),
    _waveTopColor   ( copy._waveTopColor ),
    _waveBottomColor( copy._waveBottomColor ),
    _useCrestFoam   ( copy._useCrestFoam ),
    _foamCapBottom  ( copy._foamCapBottom ),
    _foamCapTop     ( copy._foamCapTop ),
    _isStateDirty   ( copy._isStateDirty ),
    _averageHeight  ( copy._averageHeight ),
    _lightColor     ( copy._lightColor )
{}

FFTOceanSurface::~FFTOceanSurface(void)
{
}

void FFTOceanSurface::build( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurface::build()" << std::endl;

    computeSea( _NUMFRAMES );
    createOceanTiles();
    computeVertices(0);
    computePrimitives();

    initStateSet();

    _isDirty =  false;
    _isStateDirty = false;

    osg::notify(osg::INFO) << "FFTOceanSurface::build() Complete." << std::endl;
}

void FFTOceanSurface::initStateSet( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurface::initStateSet()" << std::endl;
    _stateset=new osg::StateSet;

    // Environment map    
    _stateset->addUniform( new osg::Uniform("osgOcean_EnableGlobalReflections", true ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_EnvironmentMap", ENV_MAP ) );
    _stateset->setTextureAttributeAndModes( ENV_MAP, _environmentMap.get(), osg::StateAttribute::ON );
    
    // Foam
    _stateset->addUniform( new osg::Uniform("osgOcean_EnableCrestFoam", _useCrestFoam ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamCapBottom",   _foamCapBottom ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamCapTop",      _foamCapTop ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamMap",         FOAM_MAP ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamScale",       _tileResInv*30.f ) );

    if( _useCrestFoam )
    {
        osg::Texture2D* foam_tex = createTexture("sea_foam.png", osg::Texture::REPEAT );
        _stateset->setTextureAttributeAndModes( FOAM_MAP, foam_tex, osg::StateAttribute::ON );
    }

    // Noise
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseMap",     NORMAL_MAP ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseCoords0", computeNoiseCoords( 32.f, osg::Vec2f( 2.f, 4.f), 2.f, 0.f ) ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseCoords1", computeNoiseCoords( 8.f,  osg::Vec2f(-4.f, 2.f), 1.f, 0.f ) ) );

    osg::ref_ptr<osg::Texture2D> noiseMap 
        = createNoiseMap( _noiseTileSize, _noiseWindDir, _noiseWindSpeed, _noiseWaveScale, _noiseTileRes ); 

    _stateset->setTextureAttributeAndModes( NORMAL_MAP, noiseMap, osg::StateAttribute::ON );

    // Colouring
    osg::Vec4f waveTop = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveTopColor,1.f) );
    osg::Vec4f waveBot = colorLerp(_lightColor, osg::Vec4f(), osg::Vec4f(_waveBottomColor,1.f) );

    _stateset->addUniform( new osg::Uniform("osgOcean_WaveTop", waveTop ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_WaveBot", waveBot ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FresnelMul", _fresnelMul ) );    
    _stateset->addUniform( new osg::Uniform("osgOcean_EyePosition", osg::Vec3f() ) );
    
    osg::ref_ptr<osg::Program> program = createShader();
        
    if(program.valid())
        _stateset->setAttributeAndModes( program.get(), osg::StateAttribute::ON );

    _isStateDirty = false;

    osg::notify(osg::INFO) << "FFTOceanSurface::initStateSet() Complete." << std::endl;
}

osg::ref_ptr<osg::Texture2D> FFTOceanSurface::createNoiseMap(unsigned int size, 
                                                             const osg::Vec2f& windDir, 
                                                             float windSpeed,                                         
                                                             float waveScale,
                                                             float tileResolution )
{
    osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;

    FFTSimulation noiseFFT(size, windDir, windSpeed, waveScale, tileResolution,10.f);
    noiseFFT.setTime(0.f);
    noiseFFT.computeHeights(heights);
        
    OceanTile oceanTile(heights,size,tileResolution/size);

    return oceanTile.createNormalMap();
}

void FFTOceanSurface::computeSea( unsigned int totalFrames )
{
    osg::notify(osg::INFO) << "FFTOceanSurface::computeSea("<<totalFrames<<")" << std::endl;
    osg::notify(osg::INFO) << "Mipmap Levels: " << _numLevels << std::endl;
    osg::notify(osg::INFO) << "Highest Resolution: " << _tileSize << std::endl;

    FFTSimulation FFTSim( _tileSize, _windDirection, _windSpeed, _waveScale, _tileResolution, _cycleTime );

    _mipmapData.resize( totalFrames );

    _averageHeight = 0.f;

    for( unsigned int frame = 0; frame < totalFrames; ++frame )
    {
        osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;
        osg::ref_ptr<osg::Vec2Array> displacements = NULL;
        if (_isChoppy)
            displacements = new osg::Vec2Array;

        float time = _cycleTime * ( float(frame) / float(totalFrames) );

        FFTSim.setTime( time );
        FFTSim.computeHeights( heights );

        if(_isChoppy)
            FFTSim.computeDisplacements( _choppyFactor, displacements );

        _mipmapData[frame].resize( _numLevels );

        // Level 0
        _mipmapData[frame][0] = OceanTile( heights, _tileSize, _pointSpacing, displacements );

        _mipmapData[frame][0].createNormalMap();

        _averageHeight += _mipmapData[frame][0].getAverageHeight();

        // Levels 1 -> Max Level
        for(unsigned int level = 1; level < _numLevels-1; ++level )
        {
            OceanTile& lastTile = _mipmapData[frame][level-1];

            _mipmapData[frame][level] = OceanTile( lastTile, _tileSize >> level, _tileSize/(_tileSize>>level)*_pointSpacing );
        }

        // Used for lowest resolution tile
        osg::ref_ptr<osg::FloatArray> zeroHeights = new osg::FloatArray(4);
        zeroHeights->at(0) = 0.f;
        zeroHeights->at(1) = 0.f;
        zeroHeights->at(2) = 0.f;
        zeroHeights->at(3) = 0.f;

        _mipmapData[frame][_numLevels-1] = OceanTile( zeroHeights, 1, _tileSize/(_tileSize>>(_numLevels-1))*_pointSpacing );
    }

    _averageHeight /= (float)totalFrames;

    osg::notify(osg::INFO) << "Average Height: " << _averageHeight << std::endl;
    osg::notify(osg::INFO) << "FFTOceanSurface::computeSea() Complete." << std::endl;
}

void FFTOceanSurface::createOceanTiles( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurface::createOceanTiles()" << std::endl;
    osg::notify(osg::INFO) << "Total tiles: " << _numTiles*_numTiles << std::endl;
    osg::notify(osg::INFO) << "Init level: " << _numLevels-2 << std::endl;

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
            
            MipmapGeometry* patch = new MipmapGeometry( _numLevels-2, _numLevels, 0, border );

            patch->setUseDisplayList( false );
            patch->setVertexArray( _activeVertices );
            patch->setNormalArray( _activeNormals );
            patch->setColorArray    ( colours );
            patch->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
            patch->setColorBinding( osg::Geometry::BIND_OVERALL );
            patch->setDataVariance( osg::Object::DYNAMIC );
            patch->setIdx( _numVertices );

            addDrawable( patch );

            _oceanGeom[y].push_back( patch );

            unsigned int verts = 0;
            unsigned int s = 2;

            verts = s * s;

            if(x == _numTiles-1 )                       // If on right border add extra column
                verts += s;
            if(y == _numTiles-1 )                       // If on bottom border add extra row
                verts += s;
            if(x == _numTiles-1 && y == _numTiles-1)    // If corner piece add corner vertex
                verts += 1;

            _numVertices += verts;
        }
    }

    osg::notify(osg::INFO) << "Vertices needed: " << _numVertices << std::endl;

    _activeVertices->resize( _numVertices );
    _activeNormals->resize( _numVertices );

// Correct dMin calculations for geomipmap distances. Not used at the moment
//    float T = (2.0f * TRESHOLD) / VRES;
//    float A = 1.0f / (float)tan(FOV / 2.0f);
//    float C = A / T;

    osg::notify(osg::INFO) << "Minimum Distances: " << std::endl;

    for(unsigned int d = 0; d < _numLevels; ++d)
    {
        _minDist.push_back( d * (float(_tileResolution+1)) + ( float(_tileResolution+1.f)*0.5f ) );
        _minDist.back() *= _minDist.back();
        osg::notify(osg::INFO) << d << ": " << sqrt(_minDist.back()) << std::endl;
    }

    osg::notify(osg::INFO) << "FFTOceanSurface::createOceanTiles() Complete." << std::endl;
}

void FFTOceanSurface::computeVertices( unsigned int frame )
{
    // Only resize vertex/normal arrays if more are needed
    if(_newNumVertices > _numVertices )
    {
        osg::notify(osg::INFO) << "Resizing vertex array from " << _numVertices << "to " << _newNumVertices << std::endl;
        _numVertices = _newNumVertices;
        _activeVertices->resize(_numVertices);
        _activeNormals->resize(_numVertices);
    }

    osg::Vec3f tileOffset,vertexOffset,vertex;
    
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

void FFTOceanSurface::update( unsigned int frame, const double& dt, const osg::Vec3f& eye )
{
    if(_isDirty)
        build();
    else if(_isStateDirty)
        initStateSet();

    static double time = 0.0;
    time+=(dt*0.0008);

    getStateSet()->getUniform("osgOcean_EyePosition")->set(eye);
    getStateSet()->getUniform("osgOcean_NoiseCoords0")->set( computeNoiseCoords( 32.f, osg::Vec2f( 2.f, 4.f), 2.f, time ) );
    getStateSet()->getUniform("osgOcean_NoiseCoords1")->set( computeNoiseCoords( 8.f,  osg::Vec2f(-4.f, 2.f), 1.f, time ) );
    
    if( updateMipmaps( eye, frame ) )
    {
        computeVertices( frame );
        computePrimitives();
    }
    else if( frame != _oldFrame )
    {
        computeVertices( frame );
    }

    _oldFrame = frame;
}

bool FFTOceanSurface::updateMipmaps( const osg::Vec3f& eye, unsigned int frame )
{
    static unsigned int count = 0;

    bool updated = false;

    _newNumVertices = 0;

    int tileSize = _tileResolution+1;

    int x_offset = 0;
    int y_offset = 0;

    if(_isEndless)
    {
        float xMin = _startPos.x();
        float yMin = _startPos.y() - (float)((_tileResolution+1)*_numTiles);

        x_offset = (int) ( (eye.x()-xMin) / (float)_tileResolution );
        y_offset = (int) ( (eye.y()-yMin) / (float)_tileResolution );

        x_offset -= _numTiles/2;
        y_offset -= _numTiles/2;

        _startPos.x() += (float)(x_offset * tileSize); 
        _startPos.y() += (float)(y_offset * tileSize); 
    }

    for( unsigned int y = 0; y < _numTiles; ++y)
    {
        for( unsigned int x = 0; x < _numTiles; ++x)
        {
            osg::Vec3f newbound = getTile(x,y)->getBound().center();
            newbound.x() += (float)(x_offset * tileSize);
            newbound.y() += (float)(y_offset * tileSize);

            osg::Vec3f distanceToTile = newbound - eye;
            
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

    osg::notify(osg::DEBUG_INFO) << "FFTOceanSurface::computePrimitives()" << std::endl;

    //debugOut << std::endl;

    for(unsigned int y = 0; y < _numTiles; ++y)
    {
        //osg::notify(osg::DEBUG_INFO) << std::endl;

        for(unsigned int x = 0; x < _numTiles; ++x )
        {
            osg::notify(osg::DEBUG_INFO) <<getTile(x,y)->getLevel() << " ";
            
            x+1 > _numTiles-1 ? x1 = _numTiles-1 : x1 = x+1;
            y+1 > _numTiles-1 ? y1 = _numTiles-1 : y1 = y+1;

            MipmapGeometry* cTile  = getTile(x, y);    // Current tile
            MipmapGeometry* xTile  = getTile(x1,y);    // Right Tile
            MipmapGeometry* yTile  = getTile(x, y1);   // Bottom Tile
            MipmapGeometry* xyTile = getTile(x1,y1);   // Bottom right Tile

            // First clear old primitive sets
            cTile->removePrimitiveSet(0, cTile->getNumPrimitiveSets() );

            if(cTile->getResolution()!=1)
            {
                addMainBody(cTile);

                if( x < _numTiles-1 )
                    addRightBorder( cTile, xTile );

                if( y < _numTiles-1 )
                    addBottomBorder( cTile, yTile );

                addCornerPatch( cTile, xTile, yTile, xyTile );
            }
            else
            {
                if(cTile->getBorder() == MipmapGeometry::BORDER_NONE )
                    addMaxDistMainBody( cTile, xTile, yTile, xyTile );
                else
                    addMaxDistEdge(cTile,xTile,yTile);
            }
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
            (*strip)[i]   = cTile->getIndex( col, row   );
            (*strip)[i+1] = cTile->getIndex( col, row+1 );
            i+=2;

            if( col == degenX && row+1 != degenY )
            {
                (*strip)[i]   = cTile->getIndex( col, row+1 );
                (*strip)[i+1] = cTile->getIndex( 0,   row+1 );
                i+=2;
            }
        }
    }
    cTile->addPrimitiveSet( strip );
}

void FFTOceanSurface::addMaxDistEdge(  MipmapGeometry* cTile, MipmapGeometry* xTile, MipmapGeometry* yTile )
{
    if( cTile->getBorder() == MipmapGeometry::BORDER_X )
    {
        osg::DrawElementsUInt* strip = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 4 );

        (*strip)[0] = cTile->getIndex ( 0, 0 );
        (*strip)[1] = yTile->getIndex ( 0, 0 );
        (*strip)[2] = cTile->getIndex ( 1, 0 );
        (*strip)[3] = yTile->getIndex ( 1, 0 );

        cTile->addPrimitiveSet( strip );
    }
    else if( cTile->getBorder() == MipmapGeometry::BORDER_Y )
    {
        osg::DrawElementsUInt* strip = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 4 );

        (*strip)[0] = cTile->getIndex ( 0, 0 );
        (*strip)[1] = cTile->getIndex ( 0, 1 );
        (*strip)[2] = xTile->getIndex ( 0, 0 );
        (*strip)[3] = xTile->getIndex ( 0, 1 );

        cTile->addPrimitiveSet( strip );
    }
    else if( cTile->getBorder() == MipmapGeometry::BORDER_XY )
    {
        osg::DrawElementsUInt* strip = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 4 );

        (*strip)[0] = cTile->getIndex ( 0, 0 );
        (*strip)[1] = cTile->getIndex ( 0, 1 );
        (*strip)[2] = cTile->getIndex ( 1, 0 );
        (*strip)[3] = cTile->getIndex ( 1, 1 );

        cTile->addPrimitiveSet( strip );
    }
}

void FFTOceanSurface::addMaxDistMainBody(  MipmapGeometry* cTile, MipmapGeometry* xTile, MipmapGeometry* yTile, MipmapGeometry* xyTile )
{
    int x_points = xTile->getResolution() / cTile->getResolution();
    int y_points = yTile->getResolution() / cTile->getResolution(); 

    // same res bottom and right
    if( x_points == 1 && y_points == 1)
    {
        osg::DrawElementsUInt* strip = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_STRIP, 4 );

        (*strip)[0] = cTile->getIndex ( 0, 0 );
        (*strip)[1] = yTile->getIndex ( 0, 0 );
        (*strip)[2] = xTile->getIndex ( 0, 0 );
        (*strip)[3] = xyTile->getIndex( 0, 0 );
        
        cTile->addPrimitiveSet( strip );
    }
    // high res below same res right
    else if( x_points == 1 && y_points == 2 )
    {
        osg::DrawElementsUInt* fan = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );

        (*fan)[0] = xTile->getIndex ( 0, 0 );
        (*fan)[1] = cTile->getIndex ( 0, 0 );
        (*fan)[2] = yTile->getIndex ( 0, 0 );
        (*fan)[3] = yTile->getIndex ( 1, 0 );
        (*fan)[4] = xyTile->getIndex( 0, 0 );

        cTile->addPrimitiveSet( fan );
    }
    // same res below high res below
    else if( x_points == 2 && y_points == 1 )
    {
        osg::DrawElementsUInt* fan = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 5 );

        (*fan)[0] = cTile->getIndex ( 0, 0 );
        (*fan)[1] = yTile->getIndex ( 0, 0 );
        (*fan)[2] = xyTile->getIndex( 0, 0 );
        (*fan)[3] = xTile->getIndex ( 0, 1 );
        (*fan)[4] = xTile->getIndex ( 0, 0 );

        cTile->addPrimitiveSet( fan );
    }
    // high res below and right
    else if( x_points == 2 && y_points == 2 )
    {
        osg::DrawElementsUInt* fan = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLE_FAN, 6 );

        (*fan)[0] = cTile->getIndex ( 0, 0 );
        (*fan)[1] = yTile->getIndex ( 0, 0 );
        (*fan)[2] = yTile->getIndex ( 1, 0 );
        (*fan)[3] = xyTile->getIndex( 0, 0 );
        (*fan)[4] = xTile->getIndex ( 0, 1 );
        (*fan)[5] = xTile->getIndex ( 0, 0 );

        cTile->addPrimitiveSet( fan );
    }
}

void FFTOceanSurface::addRightBorder( MipmapGeometry* cTile, MipmapGeometry* xTile )
{
    unsigned int endCol = cTile->getRowLen() - 1;

    // Same level to the right
    if( cTile->getLevel() == xTile->getLevel() )
    {
        //  3   2
        //
        //  0   1

        for(unsigned int r = 0; r < cTile->getColLen()-1; ++r)    
        {
            osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 4);

            (*fan)[0] = cTile->getIndex( endCol, r+1 );        
            (*fan)[1] = xTile->getIndex( 0,      r+1 );        
            (*fan)[2] = xTile->getIndex( 0,      r   );        
            (*fan)[3] = cTile->getIndex( endCol, r   );        

            cTile->addPrimitiveSet( fan );
        }
    }
    // low res to the right
    else if( cTile->getLevel() < xTile->getLevel() )
    {
        unsigned int diff = cTile->getResolution() / xTile->getResolution(); 
        unsigned int cPts = diff + 1;        
        unsigned int start = 0;

        //  1   0
        //  2
        //  3   4
        
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

        //  4       3
        //          2
        //  0       1

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
            (*fan)[i]   = cTile->getIndex( c, endRow );    // 0        2
            (*fan)[i+1] = yTile->getIndex( c, 0      );    // 1        3
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

        // 4    3   2
        //
        // 0        1

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

        //  1       0
        //
        // 2    3   4

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
    unsigned int botSize   = yTile->getResolution()-1;
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

                (*fan)[0] = cTile->getIndex ( curSize,   curSize   ); // 5    4
                (*fan)[1] = cTile->getIndex ( curSize-1, curSize   ); //    
                (*fan)[2] = yTile->getIndex ( botSize,   0         ); // 1    0    
                (*fan)[3] = xyTile->getIndex( 0,         0         ); // 
                (*fan)[4] = xTile->getIndex ( 0,         rightSize ); // 2         3
                (*fan)[5] = cTile->getIndex ( curSize,   curSize-1 );    

                cTile->addPrimitiveSet( fan );
            }
            // same res right
            else if( x_points == 1 )
            {
                osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 5);

                (*fan)[0] = yTile->getIndex ( botSize,   0         );    //
                (*fan)[1] = xyTile->getIndex( 0,         0         );    //           4    3    2
                (*fan)[2] = xTile->getIndex ( 0,         rightSize );    //
                (*fan)[3] = cTile->getIndex ( curSize,   curSize   );    // 0         1
                (*fan)[4] = cTile->getIndex ( curSize-1, curSize   );    //

                cTile->addPrimitiveSet( fan );
            }
            // high res right
            else if( x_points == 2 )
            {
                osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

                (*fan)[0] = yTile->getIndex    ( botSize,   0           );    // 5    4    3
                (*fan)[1] = xyTile->getIndex    ( 0,        0           );    //
                (*fan)[2] = xTile->getIndex    ( 0,         rightSize   );    //           2
                (*fan)[3] = xTile->getIndex    ( 0,         rightSize-1 );    //    
                (*fan)[4] = cTile->getIndex    ( curSize,   curSize     );    // 0         1
                (*fan)[5] = cTile->getIndex    ( curSize-1, curSize     );    

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

                (*fan)[0] = cTile->getIndex ( curSize,   curSize   ); //      6    5
                (*fan)[1] = cTile->getIndex ( curSize-1, curSize   ); //    
                (*fan)[2] = yTile->getIndex ( botSize-1, 0         ); // 1         0    
                (*fan)[3] = yTile->getIndex ( botSize,   0         ); // 
                (*fan)[4] = xyTile->getIndex( 0,         0         ); //           2    3    4
                (*fan)[5] = xTile->getIndex ( 0,         rightSize );
                (*fan)[6] = cTile->getIndex ( curSize,   curSize-1 );    

                cTile->addPrimitiveSet( fan );
            }
            // same res right
            else if( x_points == 1 )
            {
                osg::DrawElementsUInt* strip = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP, 6);

                (*strip)[0] = cTile->getIndex ( curSize-1, curSize   ); // 0    2    4
                (*strip)[1] = yTile->getIndex ( botSize-1, 0         ); //    
                (*strip)[2] = cTile->getIndex ( curSize,   curSize   ); //      1    3    5
                (*strip)[3] = yTile->getIndex ( botSize,   0         ); // 
                (*strip)[4] = xTile->getIndex ( 0,         rightSize );    
                (*strip)[5] = xyTile->getIndex( 0,         0         );

                cTile->addPrimitiveSet( strip );
            }
            // high res right
            else if( x_points == 2 )
            {
                osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 7);

                (*fan)[0] = cTile->getIndex ( curSize,   curSize     ); // 1    0    6
                (*fan)[1] = cTile->getIndex ( curSize-1, curSize     ); //    
                (*fan)[2] = yTile->getIndex ( botSize-1, 0           ); //                5
                (*fan)[3] = yTile->getIndex ( botSize,   0           ); //
                (*fan)[4] = xyTile->getIndex( 0,         0           ); //      2    3    4
                (*fan)[5] = xTile->getIndex ( 0,         rightSize   );
                (*fan)[6] = xTile->getIndex ( 0,         rightSize-1 );

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

                (*fan)[0] = xTile->getIndex( 0,         rightSize );    // 1         0
                (*fan)[1] = cTile->getIndex( curSize,   curSize-1 );    //
                (*fan)[2] = cTile->getIndex( curSize,   curSize   );    // 2            
                (*fan)[3] = yTile->getIndex( botSize-1, 0         );    //
                (*fan)[4] = yTile->getIndex( botSize,   0         );    // 3    4    5
                (*fan)[5] = xyTile->getIndex( 0,        0         );                    

                cTile->addPrimitiveSet( fan );
            }
            // same res right
            if( x_points == 1 )
            {
                osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 5);

                (*fan)[0] = xTile->getIndex ( 0,         rightSize );    // 1         0
                (*fan)[1] = cTile->getIndex ( curSize,   curSize   );    //    
                (*fan)[2] = yTile->getIndex ( botSize-1, 0         );    // 
                (*fan)[3] = yTile->getIndex ( botSize,   0         );    // 2    3    4
                (*fan)[4] = xyTile->getIndex( 0,         0         );                    

                cTile->addPrimitiveSet( fan );
            }
            // high res right
            if( x_points == 2 )
            {
                osg::DrawElementsUInt* fan = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 6);

                (*fan)[0] = cTile->getIndex ( curSize,   curSize     );    //    
                (*fan)[1] = yTile->getIndex ( botSize-1, 0           );    // 0         5
                (*fan)[2] = yTile->getIndex ( botSize,   0           );    // 
                (*fan)[3] = xyTile->getIndex( 0,         0           );    //           4
                (*fan)[4] = xTile->getIndex ( 0,         rightSize   );    //
                (*fan)[5] = xTile->getIndex ( 0,         rightSize-1 );    // 1    2    3            

                cTile->addPrimitiveSet( fan );
            }
        }
    }
}

osg::Vec3f FFTOceanSurface::computeNoiseCoords(float noiseSize, const osg::Vec2f& movement, float speed, float time )
{
    float length = noiseSize*movement.length();
    float totalTime = length / speed;    
    float tileScale = _tileResInv * noiseSize;

    osg::Vec2f velocity = movement * speed / length;
    osg::Vec2f pos = velocity * fmod( time, totalTime );

    return osg::Vec3f( pos, tileScale );
}

osg::Texture2D* FFTOceanSurface::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
    osg::Texture2D* tex = new osg::Texture2D();

    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap  (osg::Texture::WRAP_S,     wrap);
    tex->setWrap  (osg::Texture::WRAP_T,     wrap);
    tex->setImage (osgDB::readImageFile(name.c_str()));

    return tex;
}

osg::Program* FFTOceanSurface::createShader(void)
{
#if USE_LOCAL_SHADERS

    static const char ocean_surface_vertex[] = 
        "uniform mat4 osg_ViewMatrixInverse;\n"
        "\n"
        "uniform vec3 osgOcean_EyePosition;\n"
        "\n"
        "uniform vec3 osgOcean_NoiseCoords0;\n"
        "uniform vec3 osgOcean_NoiseCoords1;\n"
        "\n"
        "uniform vec4 osgOcean_WaveTop;\n"
        "uniform vec4 osgOcean_WaveBot;\n"
        "\n"
        "uniform float osgOcean_FoamScale;\n"
        "\n"
        "varying vec4 vVertex;\n"
        "varying vec4 vWorldVertex;\n"
        "varying vec3 vNormal;\n"
        "varying vec3 vViewerDir;\n"
        "varying vec3 vLightDir;\n"
        "\n"
        "varying vec3 vWorldViewDir;\n"
        "varying vec3 vWorldNormal;\n"
        "\n"
        "mat3 get3x3Matrix( mat4 m )\n"
        "{\n"
        "    mat3 result;\n"
        "\n"
        "    result[0][0] = m[0][0];\n"
        "    result[0][1] = m[0][1];\n"
        "    result[0][2] = m[0][2];\n"
        "\n"
        "    result[1][0] = m[1][0];\n"
        "    result[1][1] = m[1][1];\n"
        "    result[1][2] = m[1][2];\n"
        "\n"
        "    result[2][0] = m[2][0];\n"
        "    result[2][1] = m[2][1];\n"
        "    result[2][2] = m[2][2];\n"
        "\n"
        "    return result;\n"
        "}\n"
        "\n"
        "// -------------------------------\n"
        "//          Main Program\n"
        "// -------------------------------\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    gl_Position = ftransform();\n"
        "\n"
        "    // -----------------------------------------------------------\n"
        "\n"
        "    // In object space\n"
        "    vVertex = gl_Vertex;\n"
        "    vLightDir = normalize( vec3( gl_ModelViewMatrixInverse * ( gl_LightSource[osgOcean_LightID].position ) ) );\n"
        "    vViewerDir = gl_ModelViewMatrixInverse[3].xyz - gl_Vertex.xyz;\n"
        "    vNormal = normalize(gl_Normal);\n"
        "\n"
        "    vec4 waveColorDiff = osgOcean_WaveTop-osgOcean_WaveBot;\n"
        "\n"
        "    gl_FrontColor = waveColorDiff *\n"
        "        clamp((gl_Vertex.z + osgOcean_EyePosition.z) * 0.1111111 + vNormal.z - 0.4666667, 0.0, 1.0) + osgOcean_WaveBot;\n"
        "\n"
        "    // -------------------------------------------------------------\n"
        "\n"
        "    mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;\n"
        "    mat3 modelMatrix3x3 = get3x3Matrix( modelMatrix );\n"
        "\n"
        "    // world space\n"
        "    vWorldVertex = modelMatrix * gl_Vertex;\n"
        "    vWorldNormal = modelMatrix3x3 * gl_Normal;\n"
        "    vWorldViewDir = vWorldVertex.xyz - osgOcean_EyePosition.xyz;\n"
        "\n"
        "    // ------------- Texture Coords ---------------------------------\n"
        "\n"
        "    // Normal Map Coords\n"
        "    gl_TexCoord[0].xy = ( gl_Vertex.xy * osgOcean_NoiseCoords0.z + osgOcean_NoiseCoords0.xy );\n"
        "    gl_TexCoord[0].zw = ( gl_Vertex.xy * osgOcean_NoiseCoords1.z + osgOcean_NoiseCoords1.xy );\n"
        "    gl_TexCoord[0].y = -gl_TexCoord[0].y;\n"
        "    gl_TexCoord[0].w = -gl_TexCoord[0].w;\n"
        "\n"
        "    // Foam coords\n"
        "    gl_TexCoord[1].st = gl_Vertex.xy * osgOcean_FoamScale;\n"
        "\n"
        "    // Fog coords\n"
        "    gl_FogFragCoord = gl_Position.z;\n"
        "}\n";

    static const char ocean_surface_fragment[] = 
        "uniform bool osgOcean_EnableReflections;\n"
        "uniform bool osgOcean_EnableRefractions;\n"
        "//uniform bool osgOcean_EnableGlobalReflections;  // Unused?\n"
        "uniform bool osgOcean_EnableCrestFoam;\n"
        "\n"
        "uniform bool osgOcean_EnableDOF;\n"
        "uniform bool osgOcean_EnableGlare;\n"
        "\n"
        "uniform float osgOcean_DOF_Near;\n"
        "uniform float osgOcean_DOF_Focus;\n"
        "uniform float osgOcean_DOF_Far;\n"
        "uniform float osgOcean_DOF_Clamp;\n"
        "uniform float osgOcean_FresnelMul;\n"
        "\n"
        "uniform samplerCube osgOcean_EnvironmentMap;\n"
        "uniform sampler2D   osgOcean_ReflectionMap;\n"
        "uniform sampler2D   osgOcean_RefractionMap;\n"
        "uniform sampler2D   osgOcean_FoamMap;\n"
        "uniform sampler2D   osgOcean_NoiseMap;\n"
        "\n"
        "//uniform vec3 osgOcean_EyePosition;      // Unused?\n"
        "\n"
        "// HACK: use the 2D fogging texture here\n"
        "uniform float osgOcean_UnderwaterFogDensity;\n"
        "uniform float osgOcean_AboveWaterFogDensity;\n"
        "uniform vec4  osgOcean_UnderwaterFogColor;\n"
        "uniform vec4  osgOcean_AboveWaterFogColor;\n"
        "\n"
        "uniform mat4 osg_ViewMatrixInverse;\n"
        "\n"
        "uniform float osgOcean_FoamCapBottom;\n"
        "uniform float osgOcean_FoamCapTop;\n"
        "\n"
        "varying vec3 vNormal;\n"
        "varying vec3 vViewerDir;\n"
        "varying vec3 vLightDir;\n"
        "varying vec4 vVertex;\n"
        "varying vec4 vWorldVertex;\n"
        "\n"
        "varying vec3 vWorldViewDir;\n"
        "varying vec3 vWorldNormal;\n"
        "\n"
        "mat4 worldObjectMatrix;\n"
        "\n"
        "const float shininess = 2000.0;\n"
        "\n"
        "vec4 distortGen( vec4 v, vec3 N )\n"
        "{\n"
        "    // transposed\n"
        "    const mat4 mr = mat4( 0.5, 0.0, 0.0, 0.0,\n"
        "                          0.0, 0.5, 0.0, 0.0,\n"
        "                          0.0, 0.0, 0.5, 0.0,\n"
        "                          0.5, 0.5, 0.5, 1.0 );\n"
        "\n"
        "    mat4 texgen_matrix = mr * gl_ProjectionMatrix * gl_ModelViewMatrix;\n"
        "\n"
        "    //float disp = 8.0;\n"
        "    float disp = 4.0;\n"
        "\n"
        "    vec4 tempPos;\n"
        "\n"
        "    tempPos.xy = v.xy + disp * N.xy;\n"
        "    tempPos.z  = v.z;\n"
        "    tempPos.w  = 1.0;\n"
        "\n"
        "    return texgen_matrix * tempPos;\n"
        "}\n"
        "\n"
        "vec3 reorientate( vec3 v )\n"
        "{\n"
        "    float y = v.y;\n"
        "\n"
        "    v.y = -v.z;\n"
        "    v.z = y;\n"
        "\n"
        "    return v;\n"
        "}\n"
        "\n"
        "mat3 getLinearPart( mat4 m )\n"
        "{\n"
        "    mat3 result;\n"
        "\n"
        "    result[0][0] = m[0][0];\n"
        "    result[0][1] = m[0][1];\n"
        "    result[0][2] = m[0][2];\n"
        "\n"
        "    result[1][0] = m[1][0];\n"
        "    result[1][1] = m[1][1];\n"
        "    result[1][2] = m[1][2];\n"
        "\n"
        "    result[2][0] = m[2][0];\n"
        "    result[2][1] = m[2][1];\n"
        "    result[2][2] = m[2][2];\n"
        "\n"
        "    return result;\n"
        "}\n"
        "\n"
        "vec4 computeCubeMapColor( vec3 N, vec4 V, vec3 E )\n"
        "{\n"
        "    mat3 worldObjectMat3x3 = getLinearPart( worldObjectMatrix );\n"
        "    vec4 world_pos    = worldObjectMatrix *  V;\n"
        "\n"
        "    vec3 normal = normalize( worldObjectMat3x3 * N );\n"
        "    vec3 eye = normalize( world_pos.xyz - E );\n"
        "\n"
        "    vec3 coord = reflect( eye, normal );\n"
        "\n"
        "    vec3 reflection_vector = vec3( coord.x, coord.y, -coord.z );\n"
        "\n"
        "    return textureCube(osgOcean_EnvironmentMap, reflection_vector.xzy);\n"
        "}\n"
        "\n"
        "float calcFresnel( float dotEN, float mul )\n"
        "{\n"
        "    float fresnel = clamp( dotEN, 0.0, 1.0 ) + 1.0;\n"
        "    return pow(fresnel, -8.0) * mul;\n"
        "}\n"
        "\n"
        "float alphaHeight( float min, float max, float val)\n"
        "{\n"
        "    if(max-min == 0.0)\n"
        "        return 1.0;\n"
        "\n"
        "    return (val - min) / (max - min);\n"
        "}\n"
        "\n"
        "float computeDepthBlur(float depth, float focus, float near, float far, float clampval )\n"
        "{\n"
        "   float f;\n"
        "\n"
        "   if (depth < focus){\n"
        "      // scale depth value between near blur distance and focal distance to [-1, 0] range\n"
        "      f = (depth - focus)/(focus - near);\n"
        "   }\n"
        "   else{\n"
        "      // scale depth value between focal distance and far blur\n"
        "      // distance to [0, 1] range\n"
        "      f = (depth - focus)/(far - focus);\n"
        "\n"
        "      // clamp the far blur to a maximum blurriness\n"
        "      f = clamp(f, 0.0, clampval);\n"
        "   }\n"
        "\n"
        "   // scale and bias into [0, 1] range\n"
        "   return f * 0.5 + 0.5;\n"
        "}\n"
        "\n"
        "float luminance( vec4 color )\n"
        "{\n"
        "    return (0.3*color.r) + (0.59*color.g) + (0.11*color.b);\n"
        "}\n"
        "\n"
        "// -------------------------------\n"
        "//          Main Program\n"
        "// -------------------------------\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    vec4 final_color;\n"
        "\n"
        "    vec3 noiseNormal = vec3( texture2D( osgOcean_NoiseMap, gl_TexCoord[0].xy ) * 2.0 - 1.0 );\n"
        "    noiseNormal += vec3( texture2D( osgOcean_NoiseMap, gl_TexCoord[0].zw ) * 2.0 - 1.0 );\n"
        "\n"
        "    worldObjectMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;\n"
        "\n"
        "    if(gl_FrontFacing)\n"
        "    {\n"
        "        vec3 N = normalize( vNormal + noiseNormal );\n"
        "        vec3 L = normalize( vLightDir );\n"
        "        vec3 E = normalize( vViewerDir );\n"
        "        vec3 R = reflect( -L, N );\n"
        "\n"
        "        vec4 diffuse_color;\n"
        "        vec4 specular_color;\n"
        "\n"
        "        float lambertTerm = dot(N,L);\n"
        "\n"
        "        if( lambertTerm > 0.0 )\n"
        "        {\n"
        "            diffuse_color =  gl_LightSource[osgOcean_LightID].diffuse * lambertTerm;\n"
        "\n"
        "            float specCoeff = pow( max( dot(R, E), 0.0 ), shininess );\n"
        "\n"
        "            specular_color = gl_LightSource[osgOcean_LightID].diffuse * specCoeff * 6.0;\n"
        "        }\n"
        "\n"
        "        float dotEN = dot(E, N);\n"
        "        float dotLN = dot(L, N);\n"
        "\n"
        "        float dl = max( dotLN * abs(1.0 - max(dotEN, 0.2) ), 0.0);\n"
        "\n"
        "        vec4 refraction_color = vec4( gl_Color.rgb, 1.0 );\n"
        "\n"
        "        // To cubemap or not to cubemap that is the question\n"
        "        // projected reflection looks pretty nice anyway\n"
        "        // cubemap looks wrong with fixed skydome\n"
        "        //vec4 env_color = computeCubeMapColor(N, vWorldVertex, osgOcean_EyePosition);\n"
        "\n"
        "        vec4 env_color = texture2DProj( osgOcean_ReflectionMap, distortGen(vVertex, N) );\n"
        "\n"
        "        //env_color = vec4(1.0,0.0,0.0,1.0);\n"
        "\n"
        "        //vec4 env_color = texture2DProj( osgOcean_ReflectionMap, distortGen(vVertex, N) );\n"
        "\n"
        "        env_color.a = 1.0;\n"
        "\n"
        "        float fresnel = calcFresnel(dotEN, osgOcean_FresnelMul );\n"
        "        vec4 water_color = mix(refraction_color, env_color, fresnel) + specular_color;\n"
        "\n"
        "        final_color = water_color;\n"
        "\n"
        "        if(osgOcean_EnableReflections)\n"
        "        {\n"
        "        //    vec4 reflect_color = texture2DProj( osgOcean_ReflectionMap, distortGen(vVertex, N) );\n"
        "\n"
        "        //    if(reflect_color.a != 0.0)\n"
        "        //    {\n"
        "        //        final_color = vec4( mix( reflect_color.rgb, final_color.rgb, 0.9 ), 1.0 );\n"
        "        //    }\n"
        "        }\n"
        "\n"
        "        // Calculate luminance before foam and fog.\n"
        "        float lum = luminance(final_color);\n"
        "\n"
        "        if(osgOcean_EnableCrestFoam)\n"
        "        {\n"
        "            if( vVertex.z > osgOcean_FoamCapBottom )\n"
        "            {\n"
        "                vec4 foam_color  = texture2D( osgOcean_FoamMap, gl_TexCoord[1].st );\n"
        "\n"
        "                float alpha = alphaHeight( osgOcean_FoamCapBottom, osgOcean_FoamCapTop, vVertex.z ) * (fresnel*2.0);\n"
        "\n"
        "                final_color = mix( final_color, foam_color, alpha );\n"
        "            }\n"
        "        }\n"
        "\n"
        "        // exp2 fog\n"
        "        float fogFactor = exp2(osgOcean_AboveWaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );\n"
        "        final_color = mix( osgOcean_AboveWaterFogColor, final_color, fogFactor );\n"
        "\n"
        "        if(osgOcean_EnableGlare)\n"
        "        {\n"
        "            final_color.a = lum;\n"
        "        }\n"
        "\n"
        "        gl_FragColor = final_color;\n"
        "    }\n"
        "    else\n"
        "    {\n"
        "        vec3 E = normalize( vViewerDir );\n"
        "        vec3 N = -normalize( (vWorldNormal + noiseNormal) );\n"
        "\n"
        "        vec3 incident = normalize( vWorldViewDir );\n"
        "\n"
        "        //------ Find the reflection\n"
        "        // not really usable as we would need to use cubemap again..\n"
        "        // the ocean is blue not much to reflect back\n"
        "        //vec3 reflected = reflect( incident, -N );\n"
        "        //reflected        = reorientate( reflected );\n"
        "        //vec3 reflVec    = normalize( reflected );\n"
        "\n"
        "        //------ Find the refraction from cubemap\n"
        "        vec3 refracted = refract( incident, N, 1.3333333333 );   // 1.1 looks better? - messes up position of godrays though\n"
        "        refracted.z = refracted.z - 0.015;                       // on the fringes push it down to show base texture color\n"
        "        refracted = reorientate( refracted );\n"
        "\n"
        "        vec4 refractColor = textureCube( osgOcean_EnvironmentMap, refracted );\n"
        "\n"
        "        //------ Project texture where the light isn't internally reflected\n"
        "        if(osgOcean_EnableRefractions)\n"
        "        {\n"
        "            // if alpha is 1.0 then it's a sky pixel\n"
        "            if(refractColor.a == 1.0 )\n"
        "            {\n"
        "                vec4 env_color = texture2DProj( osgOcean_RefractionMap, distortGen(vVertex, N) );\n"
        "                refractColor.rgb = mix( refractColor.rgb, env_color.rgb, env_color.a );\n"
        "            }\n"
        "        }\n"
        "\n"
        "        // if it's not refracting in, add a bit of highlighting with fresnel\n"
        "        // FIXME: should be using the fog map\n"
        "        if( refractColor.a == 0.0 )\n"
        "        {\n"
        "            float fresnel = calcFresnel( dot(E, N), 0.7 );\n"
        "            refractColor.rgb = osgOcean_UnderwaterFogColor.rgb*fresnel + (1.0-fresnel)* refractColor.rgb;\n"
        "        }\n"
        "\n"
        "        float fogFactor = exp2(osgOcean_UnderwaterFogDensity * gl_FogFragCoord * gl_FogFragCoord );\n"
        "        final_color = mix( osgOcean_UnderwaterFogColor, refractColor, fogFactor );\n"
        "\n"
        "        if(osgOcean_EnableDOF)\n"
        "        {\n"
        "            final_color.a = computeDepthBlur( gl_FogFragCoord, osgOcean_DOF_Focus, osgOcean_DOF_Near, osgOcean_DOF_Far, osgOcean_DOF_Clamp );\n"
        "        }\n"
        "\n"
        "        gl_FragColor = final_color;\n"
        "    }\n"
        "}\n";

#else
    static const char ocean_surface_vertex[]   = "water.vert";
    static const char ocean_surface_fragment[] = "water.frag";
#endif

    osg::Program* program = ShaderManager::instance().createProgram("ocean_surface", ocean_surface_vertex, ocean_surface_fragment, !USE_LOCAL_SHADERS);
    return program;
}

void FFTOceanSurface::addResourcePaths(void)
{
    const std::string shaderPath = "resources/shaders/";
    const std::string texturePath = "resources/textures/";

    osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();

    bool shaderPathPresent = false;
    bool texturePathPresent = false;

    for(unsigned int i = 0; i < pathList.size(); ++i )
    {
        if( pathList.at(i).compare(shaderPath) == 0 )
            shaderPathPresent = true;

        if( pathList.at(i).compare(texturePath) == 0 )
            texturePathPresent = true;
    }

    if(!texturePathPresent)
        pathList.push_back(texturePath);

    if(!shaderPathPresent)
        pathList.push_back(shaderPath);
}

// -------------------------------
//     Callback implementation
// -------------------------------

FFTOceanSurface::OceanDataType::OceanDataType( FFTOceanSurface& ocean, unsigned int numFrames, unsigned int fps ):
    _oceanSurface( ocean ),
    _NUMFRAMES   ( numFrames ),
    _time        ( 0.f ),
    _FPS         ( fps ), 
    _msPerFrame  ( 1000.f/(float)fps ),
    _frame       ( 0 ),
    _oldTime     ( 0 ),
    _newTime     ( 0 )
{}

FFTOceanSurface::OceanDataType::OceanDataType( const OceanDataType& copy, const osg::CopyOp& copyop ):
    _oceanSurface( copy._oceanSurface ),
    _NUMFRAMES   ( copy._NUMFRAMES ),
    _eye         ( copy._eye ),
    _time        ( copy._time ),
    _FPS         ( copy._FPS ), 
    _msPerFrame  ( copy._msPerFrame ),
    _frame       ( copy._frame ),
    _oldTime     ( copy._oldTime ),
    _newTime     ( copy._newTime )
{}

void FFTOceanSurface::OceanDataType::updateOcean( void )
{
    _oldTime = _newTime;
    _newTime = osg::Timer::instance()->tick();

    double dt = osg::Timer::instance()->delta_m(_oldTime, _newTime);
    _time += dt;

    if( _time >= _msPerFrame )
    {
        _frame += ( _time / _msPerFrame );

        if( _frame >= _NUMFRAMES ) 
            _frame = _frame%_NUMFRAMES; 

        _time = fmod( _time, _msPerFrame );
    }
    
    _oceanSurface.update( _frame, dt, _eye );
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


FFTOceanSurface::EventHandler::EventHandler(OceanTechnique* oceanSurface):
    OceanTechnique::EventHandler(oceanSurface)
{
}

bool FFTOceanSurface::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object* object, osg::NodeVisitor* nv)
{
    // Call parent class's handle().
    OceanTechnique::EventHandler::handle(ea, aa, object, nv);

    if (ea.getHandled()) return false;

    // Now we can handle this class's events.
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYUP):
        {
            // Downcast to the concrete class we're interested in.
            FFTOceanSurface* fftSurface = dynamic_cast<FFTOceanSurface*>(_oceanSurface);
            if (!fftSurface) return false;

            // Crest foam
            if (ea.getKey() == 'f')
            {
                fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
                return true;
            }

            break;
        }
    default:
        break;
    }

    return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void FFTOceanSurface::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
    // Add parent class's keys too.
    OceanTechnique::EventHandler::getUsage(usage);

    usage.addKeyboardMouseBinding("f","Toggle crest foam");
}
