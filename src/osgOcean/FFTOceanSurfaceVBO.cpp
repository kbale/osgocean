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

#include <osgOcean/FFTOceanSurfaceVBO>
#include <osgOcean/ShaderManager>
#include <osg/io_utils>
#include <osg/Material>
#include <osg/Math>
#include <osgDB/WriteFile>

using namespace osgOcean;

#define USE_LOCAL_SHADERS 1

FFTOceanSurfaceVBO::FFTOceanSurfaceVBO( unsigned int FFTGridSize,
                                        float resolution,
                                        unsigned int numTiles, 
                                        const osg::Vec2f& windDirection,
                                        float windSpeed,
                                        float depth,
                                        float reflectionDamping,
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
    _pointSpacing   ( ((float)_tileResolution) / _tileSize ),
    _windDirection  ( windDirection ),
    _noiseWindDir   ( windDirection ),
    _windSpeed      ( windSpeed ),
    _noiseWindSpeed ( windSpeed ),
    _waveScale      ( waveScale ),
    _noiseWaveScale ( waveScale ),
    _depth          ( depth ),
    _reflDampFactor ( reflectionDamping ),
    _cycleTime      ( animLoopTime ),
    _choppyFactor   ( choppyFactor ),
    _isChoppy       ( isChoppy ),
    _isEndless      ( false ),
    _oldFrame       ( 0 ),
    _numVertices    ( 0 ),
    _newNumVertices ( 0 ),
    _fresnelMul     ( 0.7 ),
    _numLevels      ( (unsigned int) osg::round( log( (float)_tileSize) / log(2.f) )+1),
//    _numLevels      ( (unsigned int) ( log( (float)_tileSize) / log(2.f) )+1),
//    _startPos       ( -float( (_tileResolution+1)*_numTiles) * 0.5f, float( (_tileResolution+1)*_numTiles) * 0.5f ),
    _startPos       ( -float( (_tileResolution)*_numTiles) * 0.5f, float( (_tileResolution)*_numTiles) * 0.5f ),
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
    setCullCallback( new OceanAnimationCallback );
    setUpdateCallback( new OceanAnimationCallback );

    _minDist.clear();
    osg::notify(osg::INFO) << "Minimum Distances: " << std::endl;

    for(unsigned int d = 0; d < _numLevels; ++d)
    {
        _minDist.push_back( d * (float(_tileResolution+1)) + ( float(_tileResolution+1.f)*0.5f ) );
        _minDist.back() *= _minDist.back();
        osg::notify(osg::INFO) << d << ": " << sqrt(_minDist.back()) << std::endl;
    }

    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::createOceanTiles() Complete." << std::endl;

}

FFTOceanSurfaceVBO::FFTOceanSurfaceVBO( const FFTOceanSurfaceVBO& copy, const osg::CopyOp& copyop ):
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

FFTOceanSurfaceVBO::~FFTOceanSurfaceVBO(void)
{
}

void FFTOceanSurfaceVBO::build( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::build()" << std::endl;

    computeSea( _NUMFRAMES );
    createOceanTiles();
    updateLevels(osg::Vec3f(0.0f, 0.0f, 0.0f));
    updateVertices(0);

    initStateSet();

    _isDirty =  false;
    _isStateDirty = false;

    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::build() Complete." << std::endl;
}

void FFTOceanSurfaceVBO::initStateSet( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::initStateSet()" << std::endl;
    _stateset=new osg::StateSet;

    // Note that we will only set the textures in the state set if shaders are
    // enabled, otherwise the fixed pipeline will try to put the env map onto
    // the water surface, which has no texture coordinates, so the surface
    // will take the general color of the env map...

    // Environment map    
    _stateset->addUniform( new osg::Uniform("osgOcean_EnvironmentMap", ENV_MAP ) );
    if (ShaderManager::instance().areShadersEnabled())
       _stateset->setTextureAttributeAndModes( ENV_MAP, _environmentMap.get(), osg::StateAttribute::ON
                                                                                   | osg::StateAttribute::PROTECTED);
    
    // Foam
    _stateset->addUniform( new osg::Uniform("osgOcean_EnableCrestFoam", _useCrestFoam ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamCapBottom",   _foamCapBottom ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamCapTop",      _foamCapTop ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamMap",         FOAM_MAP ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_FoamScale",       _tileResInv*30.f ) );

    if( _useCrestFoam )
    {
        osg::Texture2D* foam_tex = createTexture("sea_foam.png", osg::Texture::REPEAT );
        if (ShaderManager::instance().areShadersEnabled())
           _stateset->setTextureAttributeAndModes( FOAM_MAP, foam_tex, osg::StateAttribute::ON |
                                                   osg::StateAttribute::PROTECTED);
    }

    // Noise
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseMap",     NORMAL_MAP ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseCoords0", computeNoiseCoords( 32.f, osg::Vec2f( 2.f, 4.f), 2.f, 0.f ) ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_NoiseCoords1", computeNoiseCoords( 8.f,  osg::Vec2f(-4.f, 2.f), 1.f, 0.f ) ) );

    osg::ref_ptr<osg::Texture2D> noiseMap 
        = createNoiseMap( _noiseTileSize, _noiseWindDir, _noiseWindSpeed, _noiseWaveScale, _noiseTileRes ); 

    if (ShaderManager::instance().areShadersEnabled())
    {
        _stateset->setTextureAttributeAndModes( NORMAL_MAP, noiseMap.get(), osg::StateAttribute::ON |
                                                                            osg::StateAttribute::PROTECTED);
    }

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

    // If shaders are enabled, the final color will be determined by the 
    // shader so we need a white base color. But on the fixed pipeline the
    // material color will determine the ocean surface's color.
    if (!ShaderManager::instance().areShadersEnabled())
    {
        osg::Material* mat = new osg::Material;
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4f(_waveTopColor, 1.0f));
        _stateset->setAttributeAndModes(mat, osg::StateAttribute::ON);
    }

    _isStateDirty = false;

    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::initStateSet() Complete." << std::endl;
}

osg::ref_ptr<osg::Texture2D> FFTOceanSurfaceVBO::createNoiseMap(unsigned int size, 
                                                             const osg::Vec2f& windDir, 
                                                             float windSpeed,                                         
                                                             float waveScale,
                                                             float tileResolution )
{
    osg::ref_ptr<osg::FloatArray> heights = new osg::FloatArray;

    FFTSimulation noiseFFT(size, windDir, windSpeed, _depth, _reflDampFactor, waveScale, tileResolution, 10.f);
    noiseFFT.setTime(0.f);
    noiseFFT.computeHeights(heights.get());
        
    OceanTile oceanTile(heights.get(),size,tileResolution/size);

    return oceanTile.createNormalMap();
}

void FFTOceanSurfaceVBO::computeSea( unsigned int totalFrames )
{
    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::computeSea("<<totalFrames<<")" << std::endl;
    osg::notify(osg::INFO) << "Mipmap Levels: " << _numLevels << std::endl;
    osg::notify(osg::INFO) << "Highest Resolution: " << _tileSize << std::endl;

    FFTSimulation FFTSim( _tileSize, _windDirection, _windSpeed, _depth, _reflDampFactor, _waveScale, _tileResolution, _cycleTime );

    // clear previous mipmaps (if any)
    _mipmapData.clear();
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
        FFTSim.computeHeights( heights.get() );

        if(_isChoppy)
            FFTSim.computeDisplacements( _choppyFactor, displacements.get() );

        // Level 0
        _mipmapData[frame] = OceanTile( heights.get(), _tileSize, _pointSpacing, displacements.get(), true );

        _averageHeight += _mipmapData[frame].getAverageHeight();
    }
    _averageHeight /= (float)totalFrames;

    osg::notify(osg::INFO) << "Average Height: " << _averageHeight << std::endl;
    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::computeSea() Complete." << std::endl;
}

void FFTOceanSurfaceVBO::createOceanTiles( void )
{
    osg::notify(osg::INFO) << "FFTOceanSurfaceVBO::createOceanTiles()" << std::endl;
    osg::notify(osg::INFO) << "Total tiles: " << _numTiles*_numTiles << std::endl;

    // Clear previous data if it exists
    _oceanGeom.clear();

    removeDrawables(0, getNumDrawables());

    for(int y = 0; y < (int)_numTiles; ++y )
    {
        std::vector< osg::ref_ptr<osgOcean::MipmapGeometryVBO> > tileRow(_numTiles);
        for(int x = 0; x < (int)_numTiles; ++x )
        {
            int centreX = -((int)(_numTiles*(int)_tileResolution))/2;
            int centreY =  ((int)(_numTiles*(int)_tileResolution))/2;
            osg::Vec3f offset( centreX+x*(int)_tileResolution, centreY-y*(int)_tileResolution, 0.f ); 

            osgOcean::MipmapGeometryVBO* tile = new osgOcean::MipmapGeometryVBO( _numLevels, _tileResolution );
//            tile->setVertexArray( _activeVertices );
//            tile->setNormalArray( _activeNormals );
            tile->setOffset( offset );

            osg::BoundingBoxf bb;

            bb.xMin() = (int)offset.x();
            bb.xMax() = (int)offset.x()+(int)_tileResolution;

            bb.yMin() = (int)offset.y()-(int)_tileResolution;
            bb.yMax() = (int)offset.y();

            bb.zMin() = (int)-15.f;
            bb.zMax() = (int)15.f;
 
            tile->setInitialBound(bb);
            
            tileRow.at(x)=tile;

            addDrawable( tile );

        }
        _oceanGeom.push_back(tileRow);
    }

    return;
}

void FFTOceanSurfaceVBO::setMinDistances(std::vector<float> &minDist  )
{
    if (_numLevels != minDist.size())
    {
        osg::notify(osg::WARN) << "FFTOceanSurface::setMinDistances() Incorrect Number of Levels." << std::endl;
        osg::notify(osg::WARN) << "Found " << minDist.size() << " Exepcted "
                               << _numLevels << std::endl;
        osg::notify(osg::WARN) << "Ignoring Min Distances" << std::endl;
        return;
    }
    _minDist.clear();

    osg::notify(osg::INFO) << "setting Minimum Distances: " << std::endl;

    for(unsigned int d = 0; d < _numLevels; ++d)
    {
        _minDist.push_back( minDist[d] * minDist[d] );
        osg::notify(osg::INFO) << d << ": " << sqrt(_minDist.back()) << std::endl;
    }
}

static int count = 0;

void FFTOceanSurfaceVBO::updateVertices(unsigned int frame)
{
#ifdef OSGOCEAN_TIMING
    osg::Timer_t startTime;
    osg::Timer_t endTime;
    startTime = osg::Timer::instance()->tick();
#endif /*OSTOCEAN_TIMING*/

    osg::Vec3f tileOffset;

    const OceanTile& data = _mipmapData[frame];

    for(unsigned int y = 0; y < _numTiles; ++y )
    {
        for(unsigned int x = 0; x < _numTiles; ++x )
        {
            MipmapGeometryVBO* tile = getTile(x,y);
            tile->updateFrame(data.getVertices(), data.getNormals());
        }
    }

#ifdef OSGOCEAN_TIMING
    endTime = osg::Timer::instance()->tick();
    double dt = osg::Timer::instance()->delta_m(startTime, endTime);
    fprintf(stderr, "updateVertices() time = %lf\n", dt);
#endif /*OSGOCEAN_TIMING*/
}

bool FFTOceanSurfaceVBO::updateLevels(const osg::Vec3f& eye)
{
   int x_offset = 0;
   int y_offset = 0;

   if(_isEndless)
   {
      float xMin = _startPos.x();
      float yMin = _startPos.y()-(_tileResolution*_numTiles);
      
      x_offset = (int) ( (eye.x()-xMin) / _tileResolution );
      y_offset = (int) ( (eye.y()-yMin) / _tileResolution );
      
      x_offset -= ((int)_numTiles)/2;
      y_offset -= ((int)_numTiles)/2;
//      std::cerr <<  "Offset: " << x_offset << "," << y_offset << std::endl;
//      std::cerr <<  "Start: " << _startPos.x() << "," << _startPos.y() << std::endl;
      
      if(x_offset != 0 || y_offset != 0)
      {
         //std::cerr << "Surface Move." << std::endl;
         
         while ((x_offset != 0) || (y_offset != 0))
         {
            if(x_offset < 0)
            {
               osg::Vec3f offset;
               _startPos.x() -= (int)_tileResolution;
               
               for(int r = 0; r < (int)_numTiles; ++r)
               {
                  std::vector< osg::ref_ptr<osgOcean::MipmapGeometryVBO> >& row = _oceanGeom.at(r);
                  
                  offset.x() = _startPos.x();
                  offset.y() = _startPos.y()-r*(int)_tileResolution;
                  offset.z() = 0;
                  
                  row.insert( row.begin(), row.back() );   // insert the 
                  row.pop_back(); 
                  row.front()->setOffset( offset );         // change offset
               }
               ++x_offset;
            }
            else if (x_offset > 0)
            {
               osg::Vec3f offset;
               _startPos.x() += (int)_tileResolution;
               
               for(int r = 0; r < (int)_numTiles; ++r)
               {
                  std::vector< osg::ref_ptr<osgOcean::MipmapGeometryVBO> >& row = _oceanGeom.at(r);
                  
                  offset.x() = _startPos.x() + ( (_numTiles-1)*(int)_tileResolution );
                  offset.y() = _startPos.y()-r*(int)_tileResolution;
                  offset.z() = 0;
                  
                  row.insert( row.end(), row.front() );
                  row.erase( row.begin() );
                  row.back()->setOffset( offset );
               } 
               --x_offset;                  
            }
            
            if(y_offset < 0)
            {
               _startPos.y() -= (int)_tileResolution;

               _oceanGeom.insert( _oceanGeom.end(), _oceanGeom.front() );
               _oceanGeom.erase( _oceanGeom.begin() );
               
               osg::Vec3f offset;
               
               for(int c = 0; c < (int)_numTiles; c++ )
               {
                  offset.x() = _startPos.x() + c *(int) _tileResolution;
                  offset.y() = _startPos.y()-( (_numTiles-1)*(int)_tileResolution );
                  offset.z() = 0;
                  
                  _oceanGeom.back().at(c)->setOffset(offset);
               }
               ++y_offset;
            }
            else if(y_offset > 0)
            {
               _startPos.y() += (int)_tileResolution;

               _oceanGeom.insert( _oceanGeom.begin(), _oceanGeom.back() );
               _oceanGeom.pop_back();
               
               osg::Vec3f offset;
               
               for(int c = 0; c < (int)_numTiles; c++ )
               {
                  offset.x() = _startPos.x() + c * (int)_tileResolution;
                  offset.y() = _startPos.y();
                  offset.z() = 0;
                  
                  _oceanGeom.front().at(c)->setOffset(offset);
               }
               --y_offset;
            }
         }
      }
   }
   
   unsigned updates=0;
   
   for(int r = _numTiles-1; r>=0; --r )
   {
      for(int c = _numTiles-1; c>=0; --c )
      {
         osgOcean::MipmapGeometryVBO* curGeom = _oceanGeom.at(r).at(c);
         osg::Vec3f centre = curGeom->getBound().center();
         
         float distanceToTile2 = (centre-eye).length2();
         
         unsigned mipmapLevel = 0;
         unsigned rightLevel  = 0;
         unsigned belowLevel  = 0;
         
         for( unsigned int m = 0; m < _minDist.size(); ++m )
         {
            if( distanceToTile2 > _minDist.at(m) )
               mipmapLevel = m;
         }
         
         if( c != _numTiles-1 && r != _numTiles-1 ){
            osgOcean::MipmapGeometryVBO* rightGeom = _oceanGeom.at(r).at(c+1);
            osgOcean::MipmapGeometryVBO* belowGeom = _oceanGeom.at(r+1).at(c);
            rightLevel = rightGeom->getLevel();
            belowLevel = belowGeom->getLevel();
         }
         else 
         {
            if( c != _numTiles-1 ){
               osgOcean::MipmapGeometryVBO* rightGeom = _oceanGeom.at(r).at(c+1);
               rightLevel = rightGeom->getLevel();
            }
            else{
               rightLevel = mipmapLevel;
            }
            
            if( r != _numTiles-1 ){
               osgOcean::MipmapGeometryVBO* belowGeom = _oceanGeom.at(r+1).at(c);
               belowLevel = belowGeom->getLevel();
            }
            else{
               belowLevel = mipmapLevel;
            }
         }

         if( curGeom->updatePrimitives(mipmapLevel,rightLevel,belowLevel) )
            updates++;
      }
   }

#ifdef OSGOCEAN_MIPMAP
   if (updates > 0)
   {
        std::cerr <<  "Updates: " << updates << std::endl;
        for(int r = _numTiles-1; r>=0; --r )
        {
           for(int c = _numTiles-1; c>=0; --c )
           {
              fprintf(stderr, "%d", _oceanGeom.at(r).at(c)->getLevel());
           }
           fprintf(stderr, "\n");
        }
   }
#endif /*OSGOCEAN_MIPMAP*/

   return updates > 0;
}


void FFTOceanSurfaceVBO::update( unsigned int frame, const double& dt, const osg::Vec3f& eye )
{
    if(_isDirty)
        build();
    else if(_isStateDirty)
        initStateSet();

    getStateSet()->getUniform("osgOcean_EyePosition")->set(eye);

    if (_isAnimating)
    {
        static double time = 0.0;
        time+=(dt*0.0008);

        getStateSet()->getUniform("osgOcean_NoiseCoords0")->set( computeNoiseCoords( 32.f, osg::Vec2f( 2.f, 4.f), 2.f, time ) );
        getStateSet()->getUniform("osgOcean_NoiseCoords1")->set( computeNoiseCoords( 8.f,  osg::Vec2f(-4.f, 2.f), 1.f, time ) );

        if (updateLevels(eye))
        {
           updateVertices(frame);
        } 
        else if (frame != _oldFrame)
        {
              updateVertices(frame);
        }
    }

    _oldFrame = frame;
}

float FFTOceanSurfaceVBO::getSurfaceHeightAt(float x, float y, osg::Vec3f* normal)
{

    if(_isDirty)
        build();

    // ocean surface coordinates
    float oceanX, oceanY;

    // translate x, y to oceanSurface origin coordinates
    oceanX = -_startPos.x() + x;
    oceanY =  _startPos.y() - y;

    // calculate the corresponding tile on the ocean surface
    unsigned int ix = oceanX/_tileResolution;
    unsigned int iy = oceanY/_tileResolution;

    unsigned int frame = _oldFrame;

    // Test if the tile is valid 
    if(ix < _numTiles && iy < _numTiles)
    {
        const OceanTile& data = _mipmapData[_oldFrame];

        float tile_x = oceanX - ix * (int) _tileResolution;
        float tile_y = oceanY - iy * (int) _tileResolution;

        if (normal != 0)
        {
            *normal = data.normalBiLinearInterp(tile_x, tile_y);
        }

        return data.biLinearInterp(tile_x, tile_y);
    }

    return 0.0f;
}


osg::Vec3f FFTOceanSurfaceVBO::computeNoiseCoords(float noiseSize, const osg::Vec2f& movement, float speed, double time )
{
    float length = noiseSize*movement.length();
    double totalTime = length / speed;    
    float tileScale = _tileResInv * noiseSize;

    osg::Vec2f velocity = movement * speed / length;
    osg::Vec2f pos = velocity * fmod( time, totalTime );

    return osg::Vec3f( pos, tileScale );
}

osg::Texture2D* FFTOceanSurfaceVBO::createTexture(const std::string& name, osg::Texture::WrapMode wrap)
{
    osg::Texture2D* tex = new osg::Texture2D();

    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    tex->setWrap  (osg::Texture::WRAP_S,     wrap);
    tex->setWrap  (osg::Texture::WRAP_T,     wrap);
    tex->setImage (osgDB::readImageFile(name.c_str()));

    return tex;
}

osg::Program* FFTOceanSurfaceVBO::createShader(void)
{
#if USE_LOCAL_SHADERS
    static const char ocean_surface_vertex[] = 

        "uniform mat4 osg_ViewMatrixInverse;\n"
        "uniform float osg_FrameTime;\n"
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
        "// Used to blend the waves into a sinus curve near the shore\n"
        "uniform sampler2D osgOcean_Heightmap;\n"
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
        "varying float height;\n"
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
        "uniform mat4 gl_ModelViewProjectionMatrix;\n"
        "\n"
        "// -------------------------------\n"
        "//          Main Program\n"
        "// -------------------------------\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    // Transform the vertex\n"
        "    vec4 inputVertex = gl_Vertex;\n"
        "    inputVertex.xyz += gl_Color.xyz;\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * inputVertex;\n"
        "\n"
        "    // Blend the wave into a sinus curve near the shore\n"
        "    // note that this requires a vertex shader texture lookup\n"
        "    // vertex has to be transformed a second time with the new z-value\n"
        "#if SHORETOSINUS\n"
        "    vec2 screenCoords = gl_Position.xy / gl_Position.w;\n"
        "    \n"
        "    height = pow(clamp(1.0 - texture2D(osgOcean_Heightmap, clamp(screenCoords * 0.5 + 0.5, 0.0, 1.0)).x, 0.0, 1.0), 32.0);\n"
        "\n"
//        "    inputVertex = vec4(gl_Vertex.x, \n"
//        "                       gl_Vertex.y, \n"
//        "                       mix(gl_Vertex.z, sin(osg_FrameTime), height),\n"
//        "                       gl_Vertex.w);\n"
        "    inputVertex = vec4(inputVertex.x, \n"
        "                       inputVertex.y, \n"
        "                       mix(inputVertex.z, sin(osg_FrameTime), height),\n"
        "                       inputVertex.w);\n"
        "\n"
        "    gl_Position = gl_ModelViewProjectionMatrix * inputVertex;\n"
        "#endif\n"
        "\n"
        "    // -----------------------------------------------------------\n"
        "\n"
        "    // In object space\n"
        "    vVertex = inputVertex;\n"
        "    vLightDir = normalize( vec3( gl_ModelViewMatrixInverse * ( gl_LightSource[osgOcean_LightID].position ) ) );\n"
        "    vViewerDir = gl_ModelViewMatrixInverse[3].xyz - inputVertex.xyz;\n"
        "    vNormal = normalize(gl_Normal);\n"
        "\n"
        "    vec4 waveColorDiff = osgOcean_WaveTop-osgOcean_WaveBot;\n"
        "\n"
        "    gl_FrontColor = waveColorDiff *\n"
        "        clamp((inputVertex.z + osgOcean_EyePosition.z) * 0.1111111 + vNormal.z - 0.4666667, 0.0, 1.0) + osgOcean_WaveBot;\n"
        "\n"
        "    // -------------------------------------------------------------\n"
        "\n"
        "    mat4 modelMatrix = osg_ViewMatrixInverse * gl_ModelViewMatrix;\n"
        "    mat3 modelMatrix3x3 = get3x3Matrix( modelMatrix );\n"
        "\n"
        "    // world space\n"
        "    vWorldVertex = modelMatrix * inputVertex;\n"
        "    vWorldNormal = modelMatrix3x3 * gl_Normal;\n"
        "    vWorldViewDir = vWorldVertex.xyz - osgOcean_EyePosition.xyz;\n"
        "\n"
        "    // ------------- Texture Coords ---------------------------------\n"
        "\n"
        "    // Normal Map Coords\n"
        "    gl_TexCoord[0].xy = ( inputVertex.xy * osgOcean_NoiseCoords0.z + osgOcean_NoiseCoords0.xy );\n"
        "    gl_TexCoord[0].zw = ( inputVertex.xy * osgOcean_NoiseCoords1.z + osgOcean_NoiseCoords1.xy );\n"
        "    gl_TexCoord[0].y = -gl_TexCoord[0].y;\n"
        "    gl_TexCoord[0].w = -gl_TexCoord[0].w;\n"
        "\n"
        "    // Foam coords\n"
        "    gl_TexCoord[1].st = inputVertex.xy * osgOcean_FoamScale;\n"
        "\n"
        "    // Fog coords\n"
        "    gl_FogFragCoord = gl_Position.z;\n"
        "}\n";

    static const char ocean_surface_fragment[] = 

        "uniform bool osgOcean_EnableReflections;\n"
        "uniform bool osgOcean_EnableRefractions;\n"
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
        "uniform sampler2D   osgOcean_RefractionDepthMap;\n"
        "uniform sampler2D   osgOcean_FoamMap;\n"
        "uniform sampler2D   osgOcean_NoiseMap;\n"
        "uniform sampler2D   osgOcean_Heightmap;\n"
        "\n"
        "uniform float osgOcean_UnderwaterFogDensity;\n"
        "uniform float osgOcean_AboveWaterFogDensity;\n"
        "uniform vec4  osgOcean_UnderwaterFogColor;\n"
        "uniform vec4  osgOcean_AboveWaterFogColor;\n"
        "\n"
        "uniform mat4 osg_ViewMatrixInverse;\n"
        "\n"
        "uniform mat4 osgOcean_RefractionInverseTransformation;\n"
        "\n"
        "uniform vec2 osgOcean_ViewportDimensions;\n"
        "\n"
        "uniform float osgOcean_WaterHeight;\n"
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
        "varying float height;\n"
        "\n"
        "vec4 distortGen( vec4 v, vec3 N )\n"
        "{\n"
        "    // transposed\n"
        "    const mat4 mr =\n"
        "        mat4( 0.5, 0.0, 0.0, 0.0,\n"
        "                                 0.0, 0.5, 0.0, 0.0,\n"
        "                                 0.0, 0.0, 0.5, 0.0,\n"
        "                                 0.5, 0.5, 0.5, 1.0 );\n"
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
        "float computeFogFactor( float density, float fogCoord )\n"
        "{\n"
        "    return exp2(density * fogCoord * fogCoord );\n"
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
        "        vec4 specular_color = vec4(0.0, 0.0, 0.0, 0.0);\n"
        "\n"
        "        float lambertTerm = dot(N,L);\n"
        "\n"
        "        if( lambertTerm > 0.0 )\n"
        "        {\n"
        "            float specCoeff = pow( max( dot(R, E), 0.0 ), shininess );\n"
        "            specular_color = gl_LightSource[osgOcean_LightID].diffuse * specCoeff * 6.0;\n"
        "        }\n"
        "\n"
        "        float dotEN = dot(E, N);\n"
        "        float dotLN = dot(L, N);\n"
        "\n"
        "        // Fade out the distortion along the screen edges this reduces artifacts\n"
        "        // caused by texture coordinates that are distorted out of the [0, 1] range.\n"
        "        // At very close distance to the surface the distortion artifacts still appear.\n"
        "        vec2 fade_xy = pow(abs(gl_FragCoord.xy / (osgOcean_ViewportDimensions.xy * 0.5) - 1.0), 10.0);\n"
        "        float fade = 1.0 - max(fade_xy.x , fade_xy.y);\n"
        "\n"
        "        vec4 distortedVertex = distortGen(vVertex, fade * N);\n"
        "\n"
        "        // Calculate the position in world space of the pixel on the ocean floor\n"
        "        vec4 refraction_ndc = vec4(gl_FragCoord.xy / osgOcean_ViewportDimensions, texture2DProj(osgOcean_RefractionDepthMap, distortGen(vVertex, 0.0 * N)).x, 1.0);\n"
        "        vec4 refraction_screen = refraction_ndc * 2.0 - 1.0;\n"
        "        vec4 refraction_world = osgOcean_RefractionInverseTransformation * refraction_screen;\n"
        "        refraction_world = refraction_world / refraction_world.w;\n"
        "\n"
        "        // The amount of water behind the pixel\n"
        "        // (water depth as seen from the camera position)\n"
        "        float waterDepth = distance(vWorldVertex, refraction_world);\n"
        "\n"
        "#if SHORETOSINUS\n"
        "        // The vertical distance between the ocean surface and ocean floor, this uses the projected heightmap\n"
        "        float waterHeight = (texture2DProj(osgOcean_Heightmap, distortGen(vVertex, 0.0 * N)).x) * 500.0;\n"
        "#endif\n"
        "\n"
        "        // Determine refraction color\n"
        "        vec4 refraction_color = vec4(gl_Color.rgb, 1.0 );\n"
        "\n"
        "        if(osgOcean_EnableRefractions)\n"
        "        {\n"
        "            vec4 refractionmap_color = texture2DProj(osgOcean_RefractionMap, distortedVertex );\n"
        "\n"
        "            // The amount of light extinction,\n"
        "            // higher values means that less light is transmitted through the water\n"
        "            float lightExtinction = 60.0;\n"
        "\n"
        "            vec4 waterColor = mix(refractionmap_color, refraction_color, clamp(pow(waterDepth / lightExtinction, 0.3), 0.0, 1.0));\n"
        "\n"
        "#if SHORETOSINUS\n"
        "            // Extinction level for red, green and blue light in ocean water\n"
        "            // (maybe this should be changed into a user configurable shader uniform?)\n"
        "            // Values are taken from \"Rendering Water as Post-process Effect\", Wojciech Toman\n"
        "            // http://www.gamedev.net/reference/programming/features/ppWaterRender/\n"
        "            vec4 colorExtinction = vec4(4.5, 75.0, 300.0, 1.0) * 5.0;\n"
        "\n"
        "            refraction_color = mix(waterColor, refraction_color, clamp(waterHeight / colorExtinction, 0.0, 1.0));\n"
        "#else\n"
        "            refraction_color = waterColor;\n"
        "#endif\n"
        "        }\n"
        "\n"
        "        // To cubemap or not to cubemap that is the question\n"
        "        // projected reflection looks pretty nice anyway\n"
        "        // cubemap looks wrong with fixed skydome\n"
        "        //vec4 env_color = computeCubeMapColor(N, vWorldVertex, osgOcean_EyePosition);\n"
        "\n"
        "        float fresnel = calcFresnel(dotEN, osgOcean_FresnelMul );\n"
        "        \n"
        "        vec4 env_color;\n"
        "\n"
        "        if(osgOcean_EnableReflections)\n"
        "        {\n"
        "            env_color = texture2DProj( osgOcean_ReflectionMap, distortedVertex );\n"
        "        }\n"
        "        else\n"
        "        {\n"
        "            env_color = gl_LightSource[osgOcean_LightID].diffuse;            \n"
        "        }\n"
        "        \n"
        "        final_color = mix(refraction_color, env_color, fresnel) + specular_color;\n"
        "\n"
        "        // Store the color here to compute luminance later, we don't want \n"
        "        // foam or fog to be taken into account for this calculation.\n"
        "        vec4 lumColor = final_color;\n"
        "\n"
        "        if(osgOcean_EnableCrestFoam)\n"
        "        {\n"
        "#if SHORETOSINUS\n"
        "            if( vVertex.z > osgOcean_FoamCapBottom || waterHeight < 10.0)\n"
        "            {\n"
        "                vec4 foam_color = texture2D( osgOcean_FoamMap, gl_TexCoord[1].st / 10.0);\n"
        "\n"
        "                float alpha = max(alphaHeight( osgOcean_FoamCapBottom, osgOcean_FoamCapTop, vVertex.z ) * (fresnel*2.0),\n"
        "                                  0.8 - clamp(waterHeight / 10.0, 0.0, 0.8));\n"
        "\n"
        "                final_color = final_color + (foam_color * alpha);\n"
        "            }\n"
        "#else\n"
        "            if( vVertex.z > osgOcean_FoamCapBottom )\n"
        "            {\n"
        "                vec4 foam_color = texture2D( osgOcean_FoamMap, gl_TexCoord[1].st / 10.0);\n"
        "                float alpha = alphaHeight( osgOcean_FoamCapBottom, osgOcean_FoamCapTop, vVertex.z ) * (fresnel*2.0);\n"
        "                final_color = final_color + (foam_color * alpha);\n"
        "            }\n"
        "#endif\n"
        "        }\n"
        "\n"
        "\n"
        "        // exp2 fog\n"
        "        float fogFactor = computeFogFactor( osgOcean_AboveWaterFogDensity, gl_FogFragCoord );\n"
        "        \n"
        "        final_color = mix( osgOcean_AboveWaterFogColor, final_color, fogFactor );\n"
        "\n"
        "        if(osgOcean_EnableGlare)\n"
        "        {\n"
        "            float lum = luminance(lumColor);\n"
        "            gl_FragData[1] = vec4(lum);\n"
        "        }\n"
        "\n"
        "        gl_FragData[0] = final_color;\n"
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
        "               vec4 refractColor = textureCube( osgOcean_EnvironmentMap, refracted );\n"
        "\n"
        "        //------ Project texture where the light isn't internally reflected\n"
        "        if(osgOcean_EnableRefractions)\n"
        "        {\n"
        "            // if alpha is 1.0 then it's a sky pixel\n"
        "                       if(refractColor.a == 1.0 )\n"
        "                       {\n"
        "                vec4 env_color = texture2DProj( osgOcean_RefractionMap, distortGen(vVertex, N) );\n"
        "                               refractColor.rgb = mix( refractColor.rgb, env_color.rgb, env_color.a );\n"
        "                       }\n"
        "        }\n"
        "\n"
        "        // if it's not refracting in, add a bit of highlighting with fresnel\n"
        "               if( refractColor.a == 0.0 )\n"
        "               {\n"
        "                       float fresnel = calcFresnel( dot(E, N), 0.7 );\n"
        "            refractColor.rgb = osgOcean_UnderwaterFogColor.rgb*fresnel + (1.0-fresnel)* refractColor.rgb;\n"
        "               }\n"
        "\n"
        "        float fogFactor = computeFogFactor( osgOcean_UnderwaterFogDensity, gl_FogFragCoord );\n"
        "        final_color = mix( osgOcean_UnderwaterFogColor, refractColor, fogFactor );\n"
        "\n"
        "        if(osgOcean_EnableDOF)\n"
        "        {\n"
        "            float depthBlur = computeDepthBlur( gl_FogFragCoord, osgOcean_DOF_Focus, osgOcean_DOF_Near, osgOcean_DOF_Far, osgOcean_DOF_Clamp );\n"
        "            gl_FragData[1] = vec4(depthBlur);\n"
        "        }\n"
        "\n"
        "        gl_FragData[0] = final_color;\n"
        "    }\n"
        "}\n";

#else
    static const char ocean_surface_vertex[]   = "water.vert";
    static const char ocean_surface_fragment[] = "water.frag";
#endif

    osg::Program* program = ShaderManager::instance().createProgram("ocean_surface", ocean_surface_vertex, ocean_surface_fragment, !USE_LOCAL_SHADERS);
    return program;
}

void FFTOceanSurfaceVBO::addResourcePaths(void)
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

FFTOceanSurfaceVBO::OceanDataType::OceanDataType( FFTOceanSurfaceVBO& ocean, unsigned int numFrames, unsigned int fps ):
    _oceanSurface( ocean ),
    _NUMFRAMES   ( numFrames ),
    _time        ( 0.f ),
    _FPS         ( fps ), 
    _msPerFrame  ( 1000.f/(float)fps ),
    _frame       ( 0 ),
    _oldTime     ( 0 ),
    _newTime     ( 0 )
{}

FFTOceanSurfaceVBO::OceanDataType::OceanDataType( const OceanDataType& copy, const osg::CopyOp& copyop ):
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

void FFTOceanSurfaceVBO::OceanDataType::updateOcean( void )
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

void FFTOceanSurfaceVBO::OceanAnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::ref_ptr<OceanDataType> oceanData = dynamic_cast<OceanDataType*> ( node->getUserData() );

    if( oceanData.valid() )
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


FFTOceanSurfaceVBO::EventHandler::EventHandler(OceanTechnique* oceanSurface):
    OceanTechnique::EventHandler(oceanSurface),
    _autoDirty(true)
{
}

bool FFTOceanSurfaceVBO::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object* object, osg::NodeVisitor* nv)
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
            FFTOceanSurfaceVBO* fftSurface = dynamic_cast<FFTOceanSurfaceVBO*>(_oceanSurface);
            if (!fftSurface) return false;

            // Crest foam
            if (ea.getKey() == 'f' )
            {
                fftSurface->enableCrestFoam(!fftSurface->isCrestFoamEnabled());
                osg::notify(osg::NOTICE) << "Crest foam " << (fftSurface->isCrestFoamEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // isChoppy
            if( ea.getKey() == 'p' )
            {
                fftSurface->setIsChoppy(!fftSurface->isChoppy(), _autoDirty);
                osg::notify(osg::NOTICE) << "Choppy waves " << (fftSurface->isChoppy()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Wind speed + 0.5
            if (ea.getKey() == 'W')
            {
                fftSurface->setWindSpeed(fftSurface->getWindSpeed() + 0.5, _autoDirty);
                osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
                return true;
            }
            // Wind speed - 0.5
            if (ea.getKey() == 'y')
            {
                fftSurface->setWindSpeed(fftSurface->getWindSpeed() - 0.5, _autoDirty);
                osg::notify(osg::NOTICE) << "Wind speed now " << fftSurface->getWindSpeed() << std::endl;
                return true;
            }
            // Scale factor + 1e-9
            if(ea.getKey() == 'K' )
            {
                float waveScale = fftSurface->getWaveScaleFactor();
                fftSurface->setWaveScaleFactor(waveScale+(1e-9), _autoDirty);
                osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
                return true;
            }
            // Scale factor - 1e-9
            if(ea.getKey() == 'k' )
            {
                float waveScale = fftSurface->getWaveScaleFactor();
                fftSurface->setWaveScaleFactor(waveScale-(1e-9), _autoDirty);
                osg::notify(osg::NOTICE) << "Wave scale factor now " << fftSurface->getWaveScaleFactor() << std::endl;
                return true;
            }
            // Dirty geometry
            if (ea.getKey() == 'd')
            {
                osg::notify(osg::NOTICE) << "Dirtying ocean geometry" << std::endl;
                fftSurface->dirty();
                return true;
            }
            // Toggle autoDirty, if off then individual changes will be 
            // instantaneous but the user will get no feedback until they 
            // dirty manually, if on each change will dirty automatically.
            if (ea.getKey() == 'D')
            {
                _autoDirty = !_autoDirty;
                osg::notify(osg::NOTICE) << "AutoDirty " << (_autoDirty? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Print out all current settings to the console.
            if (ea.getKey() == 'P')
            {
                osg::notify(osg::NOTICE) << "Current FFTOceanSurfaceVBO settings are:" << std::endl;
                osg::notify(osg::NOTICE) << "  Endless ocean " << (fftSurface->isEndlessOceanEnabled()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Crest foam " << (fftSurface->isCrestFoamEnabled()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Choppy waves " << (fftSurface->isChoppy()? "enabled" : "disabled") << std::endl;
                osg::notify(osg::NOTICE) << "  Choppy factor " << fftSurface->getChoppyFactor() << std::endl;
                osg::notify(osg::NOTICE) << "  Wind direction " << fftSurface->getWindDirection() << std::endl;
                osg::notify(osg::NOTICE) << "  Wind speed " << fftSurface->getWindSpeed() << std::endl;
                osg::notify(osg::NOTICE) << "  Wave scale factor " << fftSurface->getWaveScaleFactor() << std::endl;
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
void FFTOceanSurfaceVBO::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
    // Add parent class's keys too.
    OceanTechnique::EventHandler::getUsage(usage);

    usage.addKeyboardMouseBinding("f","Toggle crest foam");
    usage.addKeyboardMouseBinding("p","Toggle choppy waves (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("k","Decrease wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("K","Increase wave scale factor by 1e-9 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("y","Decrease wind speed by 0.5 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("W","Increase wind speed by 0.5 (dirties geometry if autoDirty is active)");
    usage.addKeyboardMouseBinding("d","Dirty geometry manually");
    usage.addKeyboardMouseBinding("D","Toggle autoDirty (if off, changes will require manual dirty)");
    usage.addKeyboardMouseBinding("P","Print out current ocean surface settings");
}
