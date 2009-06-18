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

#include <osgOcean/OceanScene>
#include <osgOcean/ShaderManager>

using namespace osgOcean;

#define USE_LOCAL_SHADERS 1

OceanScene::OceanScene( void ):
    _oceanSurface          ( 0 ),
    _isDirty               ( true ),
    _enableReflections     ( false ),
    _enableRefractions     ( false ),
    _enableGodRays         ( false ),
    _enableSilt            ( false ),
    _enableDOF             ( false ),
    _enableGlare           ( false ),
    _reflectionTexSize     ( 512,512 ),
    _refractionTexSize     ( 512,512 ),
    _screenDims            ( 1024,768 ),
    _sunDirection          ( 0, 0, -1 ),
    _reflectionUnit        ( 1 ),
    _refractionUnit        ( 2 ),
    _reflectionSceneMask   ( 0x1 ),   // 1
    _refractionSceneMask   ( 0x2 ),   // 2
    _normalSceneMask       ( 0x4 ),   // 4
    _surfaceMask           ( 0x8 ),   // 8
    _siltMask              ( 0x10 ),  // 16
    _lightID               ( 0 ),
    _dofNear               ( 0.f ),
    _dofFar                ( 160.f ),
    _dofFocus              ( 30.f ),
    _dofFarClamp           ( 1.f ),
    _glareThreshold        ( 0.9 ),
    _glareAttenuation      ( 0.75 ),
    _underwaterFogColor    ( 0.2274509f, 0.4352941f, 0.7294117f, 1.f ),
    _underwaterAttenuation ( 0.015f, 0.0075f, 0.005f),
    _underwaterFogDensity  ( 0.01f*0.01*1.442695f ),
    _aboveWaterFogDensity  ( 0.0012f*0.0012f*1.442695f),
    _surfaceStateSet       ( new osg::StateSet ),
    _reflectionMatrix      ( 1,  0,  0,  0,
                             0,  1,  0,  0,
                             0,  0, -1,  0,    
                             0,  0,  0,  1 )
{
    addResourcePaths();

    setNumChildrenRequiringUpdateTraversal(1);
    
    _defaultSceneShader = createDefaultSceneShader();
    ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
}

OceanScene::OceanScene( OceanTechnique* technique ):
    _oceanSurface          ( technique ),
    _isDirty               ( true ),
    _enableReflections     ( false ),
    _enableRefractions     ( false ),
    _enableGodRays         ( false ),
    _enableSilt            ( false ),
    _enableDOF             ( false ),
    _enableGlare           ( false ),
    _reflectionTexSize     ( 512,512 ),
    _refractionTexSize     ( 512,512 ),
    _screenDims            ( 1024,768 ),
    _sunDirection          ( 0,0,-1 ),
    _reflectionUnit        ( 1 ),
    _refractionUnit        ( 2 ),
    _reflectionSceneMask   ( 0x1 ),
    _refractionSceneMask   ( 0x2 ),
    _normalSceneMask       ( 0x4 ),
    _surfaceMask           ( 0x8 ),
    _siltMask              ( 0x10 ), 
    _lightID               ( 0 ),
    _dofNear               ( 0.f ),
    _dofFar                ( 160.f ),
    _dofFocus              ( 30.f ),
    _dofFarClamp           ( 1.f ),
    _glareThreshold        ( 0.9f ),
    _glareAttenuation      ( 0.75f ),
    _underwaterFogColor    ( 0.2274509f, 0.4352941f, 0.7294117f, 1.f ),
    _underwaterAttenuation ( 0.015f, 0.0075f, 0.005f),
    _underwaterFogDensity  ( -0.01f*0.01*1.442695f ),
    _aboveWaterFogDensity  ( -0.0012f*0.0012f*1.442695f),
    _surfaceStateSet       ( new osg::StateSet ),
    _reflectionMatrix      ( 1,  0,  0,  0,
                             0,  1,  0,  0,
                             0,  0, -1,  0,    
                             0,  0,  0,  1 )
{
    _oceanSurface->setNodeMask( _surfaceMask );
    addChild( _oceanSurface.get() );

    addResourcePaths();

    setNumChildrenRequiringUpdateTraversal(1);

    ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", _lightID);
    _defaultSceneShader = createDefaultSceneShader();
}

OceanScene::OceanScene( const OceanScene& copy, const osg::CopyOp& copyop ):
    osg::Group             ( copy, copyop ),
    _oceanSurface          ( copy._oceanSurface ),
    _isDirty               ( copy._isDirty ),
    _enableReflections     ( copy._enableReflections ),
    _enableRefractions     ( copy._enableRefractions ),
    _enableGodRays         ( copy._enableGodRays ),
    _enableSilt            ( copy._enableSilt ),
    _enableDOF             ( copy._enableDOF ),
    _enableGlare           ( copy._enableGlare ),
    _reflectionTexSize     ( copy._reflectionTexSize ),
    _refractionTexSize     ( copy._refractionTexSize ),
    _screenDims            ( copy._screenDims ),
    _reflectionUnit        ( copy._reflectionUnit ),
    _refractionUnit        ( copy._refractionUnit ),
    _reflectionSceneMask   ( copy._reflectionSceneMask ),
    _refractionSceneMask   ( copy._refractionSceneMask ),
    _siltMask              ( copy._siltMask ),
    _surfaceMask           ( copy._surfaceMask ),
    _normalSceneMask       ( copy._normalSceneMask),
    _reflectionCamera      ( copy._reflectionCamera ),
    _refractionCamera      ( copy._refractionCamera ),
    _godrayPreRender       ( copy._godrayPreRender ),
    _godrayPostRender      ( copy._godrayPostRender ),
    _godrays               ( copy._godrays ),
    _godRayBlendSurface    ( copy._godRayBlendSurface ),
    _siltClipNode          ( copy._siltClipNode ),
    _reflectionClipNode    ( copy._reflectionClipNode ),
    _lightID               ( copy._lightID ),
    _reflectionMatrix      ( copy._reflectionMatrix),
    _surfaceStateSet       ( copy._surfaceStateSet ),
    _underwaterFogDensity  ( copy._underwaterFogDensity ),
    _aboveWaterFogDensity  ( copy._aboveWaterFogDensity ), 
    _underwaterFogColor    ( copy._underwaterFogColor ),
    _aboveWaterFogColor    ( copy._aboveWaterFogColor ),
    _underwaterDiffuse     ( copy._underwaterDiffuse ),
    _underwaterAttenuation ( copy._underwaterAttenuation ),
    _sunDirection          ( copy._sunDirection ),
    _dofNear               ( copy._dofNear ),
    _dofFar                ( copy._dofFar ),
    _dofFocus              ( copy._dofFocus ),
    _dofFarClamp           ( copy._dofFarClamp ),
    _dofPasses             ( copy._dofPasses ),
    _glarePasses           ( copy._glarePasses ),
    _dofStateSet           ( copy._dofStateSet ),
    _glareThreshold        ( copy._glareThreshold ),
    _glareAttenuation      ( copy._glareAttenuation ),
    _glareStateSet         ( copy._glareStateSet ),
    _distortionSurface     ( copy._distortionSurface),
    _globalStateSet        ( copy._globalStateSet ),
    _defaultSceneShader    ( copy._defaultSceneShader )
{
}

OceanScene::~OceanScene( void )
{

}

void OceanScene::init( void )
{
    osg::notify(osg::INFO) << "OceanScene::init()" << std::endl;

    _refractionCamera = NULL;
    _reflectionCamera = NULL;
    _godrayPreRender  = NULL;
    _godrayPostRender = NULL;

    _reflectionClipNode = NULL;

    _dofPasses.clear();
    _dofStateSet = NULL;

    _glarePasses.clear();
    _glareStateSet = NULL;
    
    _distortionSurface = NULL;

    if( _siltClipNode.valid() ){
        removeChild( _siltClipNode );
        _siltClipNode = NULL;
    }

    if( _siltClipNode.valid() ){
        removeChild( _siltClipNode );
        _siltClipNode = NULL;
    }

    if( _oceanSurface.valid() )
    {
        _globalStateSet = new osg::StateSet;

        // This is now a #define, added by the call to 
        // ShaderManager::setGlobalDefinition() in the constructors above. 
        // Note that since _lightID can change, we will need to change the
        // global definition and reload all the shaders that depend on its
        // value when it does. This is not implemented yet.
        //_globalStateSet->addUniform( new osg::Uniform("osgOcean_LightID", _lightID ) );

        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableDOF", _enableDOF ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", _enableGlare ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_EyeUnderwater", false ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_Eye", osg::Vec3f() ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_WaterHeight", _oceanSurface->getSurfaceHeight() ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogColor", _underwaterFogColor ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogColor", _aboveWaterFogColor ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterFogDensity", _underwaterFogDensity ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_AboveWaterFogDensity", _aboveWaterFogDensity ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterDiffuse", _underwaterDiffuse ) );
        _globalStateSet->addUniform( new osg::Uniform("osgOcean_UnderwaterAttenuation", _underwaterAttenuation ) );

        if(_enableDefaultShader)
        {
            _globalStateSet->setAttributeAndModes( _defaultSceneShader, osg::StateAttribute::ON );
        }

        _surfaceStateSet = new osg::StateSet;
        _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableReflections", _enableReflections ) );
        _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_EnableRefractions", _enableRefractions ) );
        _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_ReflectionMap", _reflectionUnit ) );    
        _surfaceStateSet->addUniform( new osg::Uniform("osgOcean_RefractionMap", _refractionUnit ) );
        
        if( _enableReflections )
        {
            osg::ref_ptr<osg::Texture2D> reflectionTexture = createTexture2D( _reflectionTexSize, GL_RGBA );
            
            // clip everything below water line
            _reflectionCamera=renderToTexturePass( reflectionTexture );
            _reflectionCamera->setClearColor( osg::Vec4( 0.0, 0.0, 0.0, 0.0 ) );
            _reflectionCamera->setCullMask( _reflectionSceneMask );
            _reflectionCamera->setCullCallback( new CameraCullCallback(this) );
            _reflectionCamera->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+0, osg::StateAttribute::ON );
            _reflectionCamera->getOrCreateStateSet()->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );

            _surfaceStateSet->setTextureAttributeAndModes( _reflectionUnit, reflectionTexture, osg::StateAttribute::ON );

            osg::ClipPlane* reflClipPlane = new osg::ClipPlane(0);
            reflClipPlane->setClipPlane( 0.0, 0.0, 1.0, _oceanSurface->getSurfaceHeight() );
            _reflectionClipNode = new osg::ClipNode;
            _reflectionClipNode->addClipPlane( reflClipPlane );

            addChild( _reflectionClipNode );
        }

        if( _enableRefractions )
        {
            osg::Texture2D* refractionTexture = createTexture2D( _refractionTexSize, GL_RGBA );
            
            _refractionCamera=renderToTexturePass( refractionTexture );
            _refractionCamera->setClearColor( osg::Vec4( 0.160784, 0.231372, 0.325490, 0.0 ) );
            _refractionCamera->setCullMask( _refractionSceneMask );
            _refractionCamera->setCullCallback( new CameraCullCallback(this) );

            _surfaceStateSet->setTextureAttributeAndModes( _refractionUnit, refractionTexture, osg::StateAttribute::ON );
        }

        if( _enableGodRays )
        {
            osg::TextureRectangle* godRayTexture = createTextureRectangle( _screenDims/2, GL_RGB );

            _godrays = new GodRays(10,_sunDirection, _oceanSurface->getSurfaceHeight() );
            
            _godrayPreRender=renderToTexturePass( godRayTexture );
            _godrayPreRender->setClearColor( osg::Vec4(0.0745098, 0.10588235, 0.1529411, 1.0) );
            _godrayPreRender->addChild( _godrays );
        
            _godRayBlendSurface = new GodRayBlendSurface( osg::Vec3f(-1.f,-1.f,-1.f), osg::Vec2f(2.f,2.f), godRayTexture );

            _godRayBlendSurface->setSunDirection(_sunDirection);
            _godRayBlendSurface->setEccentricity(0.3f);
            _godRayBlendSurface->setIntensity(0.1f);

            _godrayPostRender=godrayFinalPass();
            _godrayPostRender->addChild( _godRayBlendSurface );
        }

        if( _enableDOF )
        {
            _dofPasses.clear();

            osg::Vec2s lowResDims = _screenDims/4;

            _dofStateSet = new osg::StateSet;
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Near",  _dofNear ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Far",   _dofFar ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Clamp", _dofFarClamp ) );
            _dofStateSet->addUniform( new osg::Uniform("osgOcean_DOF_Focus", _dofFocus ) );

            // First capture screen
            osg::TextureRectangle* fullScreenTexture = createTextureRectangle( _screenDims, GL_RGBA );
            osg::Camera* fullPass = renderToTexturePass( fullScreenTexture );
            fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
            fullPass->setStateSet(_dofStateSet);
            _dofPasses.push_back( fullPass );

            // Downsize image
            osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back( downsamplePass( fullScreenTexture, downsizedTexture, false ) );
            
            // Gaussian blur 1
            osg::TextureRectangle* gaussianTexture_1 = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back(gaussianPass(downsizedTexture, gaussianTexture_1, true ) );

            // Gaussian blur 2
            osg::TextureRectangle* gaussianTexture_2 = createTextureRectangle( lowResDims, GL_RGBA );
            _dofPasses.push_back(gaussianPass(gaussianTexture_1, gaussianTexture_2, false ) );

            // Combiner
            osg::TextureRectangle* combinedTexture = createTextureRectangle( _screenDims, GL_RGBA );
            _dofPasses.push_back(dofCombinerPass(fullScreenTexture, gaussianTexture_2, combinedTexture ) );

            // Post render pass
            _dofPasses.push_back( dofFinalPass( combinedTexture ) );
        }

        if( _enableGlare )
        {
            _glarePasses.clear();

            osg::Vec2s lowResDims = _screenDims/4;

            _glareStateSet = new osg::StateSet;
            _glareStateSet->addUniform( new osg::Uniform("osgOcean_EnableGlare", _enableGlare ) );

            // First capture screen
            osg::TextureRectangle* fullScreenTexture = createTextureRectangle( _screenDims, GL_RGBA );
            osg::Camera* fullPass = renderToTexturePass( fullScreenTexture );
            fullPass->setCullCallback( new PrerenderCameraCullCallback(this) );
            fullPass->setStateSet(_glareStateSet);
            _glarePasses.push_back( fullPass );

            // Downsize image
            osg::TextureRectangle* downsizedTexture = createTextureRectangle( lowResDims, GL_RGBA );
            _glarePasses.push_back( downsamplePass( fullScreenTexture, downsizedTexture, true ) );

            // Streak filter top Right
            osg::TextureRectangle* streakBuffer1 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer1, 1, osg::Vec2f(0.5f,0.5f) ) );

            osg::TextureRectangle* streakBuffer2 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer1,streakBuffer2, 2, osg::Vec2f(0.5f,0.5f) ) );

            // Streak filter Bottom left
            osg::TextureRectangle* streakBuffer3 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer3, 1, osg::Vec2f(-0.5f,-0.5f) ) );

            osg::TextureRectangle* streakBuffer4 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer3,streakBuffer4, 2, osg::Vec2f(-0.5f,-0.5f) ) );

            // Streak filter Bottom right
            osg::TextureRectangle* streakBuffer5 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer5, 1, osg::Vec2f(0.5f,-0.5f) ) );

            osg::TextureRectangle* streakBuffer6 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer5,streakBuffer6, 2, osg::Vec2f(0.5f,-0.5f) ) );

            // Streak filter Top Left
            osg::TextureRectangle* streakBuffer7 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(downsizedTexture,streakBuffer7,1, osg::Vec2f(-0.5f,0.5f) ) );

            osg::TextureRectangle* streakBuffer8 = createTextureRectangle( lowResDims, GL_RGB );
            _glarePasses.push_back( glarePass(streakBuffer7,streakBuffer8, 2, osg::Vec2f(-0.5f,0.5f) ) );

            // Final pass - combine glare and blend into scene.
            _glarePasses.push_back( glareCombinerPass(fullScreenTexture, streakBuffer2, streakBuffer4, streakBuffer6, streakBuffer8 ) );
        }

        if( _enableSilt )
        {
            SiltEffect* silt = new SiltEffect;
            // Clip silt above water level
            silt->getOrCreateStateSet()->setMode( GL_CLIP_PLANE0+1, osg::StateAttribute::ON );
            silt->setIntensity(0.07f);
            silt->setParticleSpeed(0.025f);
            silt->setNodeMask(_siltMask);

            osg::ClipPlane* siltClipPlane = new osg::ClipPlane(1);
            siltClipPlane->setClipPlane( 0.0, 0.0, -1.0, _oceanSurface->getSurfaceHeight() );

            _siltClipNode = new osg::ClipNode;
            _siltClipNode->addClipPlane( siltClipPlane );
            _siltClipNode->addChild( silt );

            addChild( _siltClipNode );
        }
    }
    
    _isDirty = false;
}

void OceanScene::traverse( osg::NodeVisitor& nv )
{
    if( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
        if( _isDirty )
            init();

        update(nv);

        osg::Group::traverse(nv);
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

        if (cv) 
        {
            preRenderCull(*cv);     // reflections/refractions
            
            // Above water
            if( cv->getEyePoint().z() > _oceanSurface->getSurfaceHeight() ) {
                if(!_enableGlare)     
                    cull(*cv);        // normal scene render
            }
            // Below water passes
            else {
                    if(!_enableDOF)
                    cull(*cv);        // normal scene render
            }

            postRenderCull(*cv);    // god rays/dof/glare
        }
        else
            osg::Group::traverse(nv);
    }
    else
        osg::Group::traverse(nv);
}

void OceanScene::update( osg::NodeVisitor& nv )
{
    if( _godrays.valid() )
        _godrays->accept(nv);

    if( _godRayBlendSurface.valid() )
        _godRayBlendSurface->accept(nv);

    if( _distortionSurface.valid() )
        _distortionSurface->accept(nv);
}

void OceanScene::preRenderCull( osgUtil::CullVisitor& cv )
{
    // Above water
    if( cv.getEyePoint().z() > _oceanSurface->getSurfaceHeight() )
    {
        if( _reflectionCamera.valid() )    
        {
            // update reflection camera and render reflected scene
            _reflectionCamera->setViewMatrix( _reflectionMatrix * cv.getCurrentCamera()->getViewMatrix() );
            _reflectionCamera->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );
            
            cv.pushStateSet(_globalStateSet);
            _reflectionCamera->accept( cv );
            cv.popStateSet();
        }

        if( _enableGlare )
        {
            // set view and projection to match main camera
            _glarePasses.at(0)->setViewMatrix( cv.getCurrentCamera()->getViewMatrix() );
            _glarePasses.at(0)->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );

            for( unsigned int i=0; i<_glarePasses.size()-1; ++i )
            {
                _glarePasses.at(i)->accept(cv);
            }
        }
    }
    // Below water
    else
    {
        if( _refractionCamera.valid() )
        {
            // update refraction camera and render refracted scene
            _refractionCamera->setViewMatrix( cv.getCurrentCamera()->getViewMatrix() );
            _refractionCamera->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );
            cv.pushStateSet(_globalStateSet);
            _refractionCamera->accept( cv );
            cv.popStateSet();
        }

        if( _godrayPreRender.valid() )
        {
            // Render the god rays to texture
            _godrayPreRender->setViewMatrix( cv.getCurrentCamera()->getViewMatrix() );
            _godrayPreRender->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );
            _godrayPreRender->accept( cv );
        }

        if( _enableDOF )
        {
            // set view and projection to match main camera
            _dofPasses.at(0)->setViewMatrix( cv.getCurrentCamera()->getViewMatrix() );
            _dofPasses.at(0)->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );

            // pass the cull visitor down the chain
            for(unsigned int i = 0; i<_dofPasses.size()-1; ++i)
            {
                _dofPasses.at(i)->accept(cv);
            }
        }
    }
}

void OceanScene::postRenderCull( osgUtil::CullVisitor& cv )
{
    if( cv.getEyePoint().z() < _oceanSurface->getSurfaceHeight() )
    {
        // dof screen first
        if( _enableDOF )
        {
            _dofPasses.back()->accept(cv);
        }
        // blend godrays ontop
        if( _enableGodRays )
        {
            _godrayPostRender->accept(cv);
        }
    }
    else
    {
        if( _enableGlare )
        {
            _glarePasses.back()->accept(cv);
        }
    }
}

void OceanScene::cull(osgUtil::CullVisitor& cv)
{
    if( _oceanSurface.valid() )
    {
          if(cv.getEyePoint().z() < _oceanSurface->getSurfaceHeight() )
          {
            _globalStateSet->getUniform("osgOcean_Eye")->set( cv.getEyePoint() );
            _globalStateSet->getUniform("osgOcean_EyeUnderwater")->set(true);
          }
        else
            _globalStateSet->getUniform("osgOcean_EyeUnderwater")->set(false);

        unsigned int mask = cv.getTraversalMask();

        cv.pushStateSet(_globalStateSet);

        // render ocean surface with reflection / refraction stateset
        cv.pushStateSet( _surfaceStateSet );
        cv.setTraversalMask( mask & _surfaceMask );
        osg::Group::traverse(cv);
        
        // pop surfaceStateSet
        cv.popStateSet(); 

        // render rest of scene
        cv.setTraversalMask( mask & _normalSceneMask );
        osg::Group::traverse(cv);

        // pop globalStateSet
        cv.popStateSet(); 

        if( cv.getEyePoint().z() < _oceanSurface->getSurfaceHeight() )
        {
            if( _enableSilt )
            {
                cv.setTraversalMask( mask & _siltMask );
                osg::Group::traverse(cv);
            }
        }

        // put original mask back
        cv.setTraversalMask( mask );
    }
    else
        osg::Group::traverse(cv);
}

osg::Camera* OceanScene::renderToTexturePass( osg::Texture* textureBuffer )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    camera->setClearColor( osg::Vec4f(0.f, 0.f, 0.f, 1.f) );
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
    camera->setViewport( 0,0, textureBuffer->getTextureWidth(), textureBuffer->getTextureHeight() );
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->attach( osg::Camera::COLOR_BUFFER, textureBuffer );

    return camera;
}

osg::Camera* OceanScene::godrayFinalPass( void )
{
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( -1.f, 1.f, -1.f, 1.f, 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );
    
    return camera;
}

osg::Camera* OceanScene::downsamplePass(osg::TextureRectangle* inputTexture, 
                                                     osg::TextureRectangle* outputTexture,
                                                     bool isGlareEffect )
{
#if USE_LOCAL_SHADERS
    static const char downsample_vertex[] = 
        "void main( void )\n"
        "{\n"
        "   gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "   gl_Position = ftransform();\n"
        "}\n";

    static const char downsample_fragment[] = 
        "uniform sampler2DRect osgOcean_GlareTexture;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    vec2 texCoordSample = vec2(0.0);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x - 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y + 1;\n"
        "    vec4 color = texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x + 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y + 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x + 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y - 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x - 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y - 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    gl_FragColor = color * 0.25;\n"
        "}\n";

    static const char downsample_glare_fragment[] = 
        "uniform sampler2DRect osgOcean_GlareTexture;\n"
        "uniform float osgOcean_GlareThreshold;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    vec2 texCoordSample = vec2(0.0);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x - 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y + 1;\n"
        "    vec4 color = texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x + 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y + 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x + 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y - 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "    texCoordSample.x = gl_TexCoord[0].x - 1;\n"
        "    texCoordSample.y = gl_TexCoord[0].y - 1;\n"
        "    color += texture2DRect(osgOcean_GlareTexture, texCoordSample);\n"
        "\n"
        "  color = color*0.25;\n"
        "\n"
        "  if(color.a >= osgOcean_GlareThreshold) "
        "      gl_FragColor = color;\n"
        "  else\n"
        "    gl_FragColor = vec4(0.0);"
        "\n"  
        "}\n";
#else
    static const char downsample_vertex[] = "downsample.vert";
    static const char downsample_fragment[] = "downsample.frag";
    static const char downsample_glare_fragment[] = "downsample_glare.frag";
#endif
    osg::Vec2s lowResDims = _screenDims/4.f;

    osg::StateSet* ss = new osg::StateSet;

    if(isGlareEffect){
        ss->setAttributeAndModes( ShaderManager::instance().createProgram("downsample_glare", downsample_vertex, downsample_glare_fragment, !USE_LOCAL_SHADERS ), osg::StateAttribute::ON );
        ss->addUniform( new osg::Uniform("osgOcean_GlareThreshold", _glareThreshold ) );
    }
    else
        ss->setAttributeAndModes( ShaderManager::instance().createProgram("downsample", downsample_vertex, downsample_fragment, !USE_LOCAL_SHADERS ), osg::StateAttribute::ON );

    ss->setTextureAttributeAndModes(0, inputTexture, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform( "osgOcean_GlareTexture", 0 ) );

    osg::Geode* downSizedQuad = createScreenQuad( lowResDims, _screenDims );
    downSizedQuad->setStateSet(ss);

    osg::Camera* RTTCamera = renderToTexturePass( outputTexture );
    RTTCamera->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
    RTTCamera->setViewMatrix(osg::Matrix::identity());
    RTTCamera->addChild( downSizedQuad );

    return RTTCamera;
}

osg::Camera* OceanScene::gaussianPass( osg::TextureRectangle* inputTexture, osg::TextureRectangle* outputTexture, bool isXAxis )
{
#if USE_LOCAL_SHADERS
    static const char gaussian_vertex[] = 
        "void main(void)\n"
        "{\n"
        "   gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "   gl_Position = ftransform();\n"
        "}\n"
        "\n";

    static const char gaussian1_fragment[] = 
        "uniform sampler2DRect osgOcean_GaussianTexture;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "   vec2 texCoordSample = vec2( 0.0 );\n"
        "   \n"
        "   vec4 color = 0.5 * texture2DRect( osgOcean_GaussianTexture, gl_TexCoord[0].st );\n"
        "   \n"
        "   texCoordSample.x = gl_TexCoord[0].x;\n"
        "   texCoordSample.y = gl_TexCoord[0].y + 1;\n"
        "   color += 0.25 * texture2DRect( osgOcean_GaussianTexture, texCoordSample);\n"
        "   \n"
        "   texCoordSample.y = gl_TexCoord[0].y - 1;\n"
        "   color += 0.25 * texture2DRect( osgOcean_GaussianTexture, texCoordSample);\n"
        "\n"
        "   gl_FragColor = color;\n"
        "}\n";

    static const char gaussian2_fragment[] = 
        "uniform sampler2DRect osgOcean_GaussianTexture;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "   vec2 texCoordSample = vec2( 0.0 );\n"
        "   \n"
        "   vec4 color = 0.5 * texture2DRect(osgOcean_GaussianTexture, gl_TexCoord[0].st );\n"
        "   \n"
        "   texCoordSample.y = gl_TexCoord[0].y;\n"
        "   texCoordSample.x = gl_TexCoord[0].x + 1;\n"
        "   color += 0.25 * texture2DRect(osgOcean_GaussianTexture, texCoordSample);\n"
        "   \n"
        "   texCoordSample.x = gl_TexCoord[0].x - 1;\n"
        "   color += 0.25 * texture2DRect(osgOcean_GaussianTexture, texCoordSample);\n"
        "      \n"
        "   gl_FragColor = color;\n"
        "}\n";
#else
    static const char gaussian_vertex[] = "gaussian1.vert";
    static const char gaussian1_fragment[] = "gaussian1.frag";
    static const char gaussian2_fragment[] = "gaussian2.frag";
#endif

    osg::Vec2s lowResDims = _screenDims/4.f;

    osg::StateSet* ss = new osg::StateSet;
    
    if(isXAxis)
        ss->setAttributeAndModes( ShaderManager::instance().createProgram("gaussian1", gaussian_vertex, gaussian1_fragment, !USE_LOCAL_SHADERS ), osg::StateAttribute::ON );
    else
        ss->setAttributeAndModes( ShaderManager::instance().createProgram("gaussian2", gaussian_vertex, gaussian2_fragment, !USE_LOCAL_SHADERS ), osg::StateAttribute::ON );

    ss->setTextureAttributeAndModes( 0, inputTexture, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform( "osgOcean_GaussianTexture", 0 ) );

    osg::Geode* gaussianQuad = createScreenQuad( lowResDims, lowResDims );
    gaussianQuad->setStateSet(ss);

    osg::Camera* dofGaussianPass = renderToTexturePass( outputTexture );
    dofGaussianPass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0, lowResDims.y(), 1, 10 );
    dofGaussianPass->addChild(gaussianQuad);

    return dofGaussianPass;
}

osg::Camera* OceanScene::dofCombinerPass(osg::TextureRectangle* fullscreenTexture, 
                                                      osg::TextureRectangle* blurTexture,
                                                      osg::TextureRectangle* outputTexture )
{
#if USE_LOCAL_SHADERS
    static const char dof_composite_vertex[] = 
        "uniform vec2 osgOcean_ScreenRes;\n"
        "uniform vec2 osgOcean_LowRes;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    gl_TexCoord[0] = gl_MultiTexCoord0 * vec4( osgOcean_ScreenRes, 1.0, 1.0 );\n"
        "    gl_TexCoord[1] = gl_MultiTexCoord0 * vec4( osgOcean_LowRes,    1.0, 1.0 );\n"
        "\n"
        "    gl_Position = ftransform();\n"
        "}\n";

    static const char dof_composite_fragment[] = 
        "uniform sampler2DRect osgOcean_FullColourMap;    // full resolution image\n"
        "uniform sampler2DRect osgOcean_BlurMap;          // downsampled and filtered image\n"
        "\n"
        "uniform vec2 osgOcean_ScreenRes;\n"
        "uniform vec2 osgOcean_ScreenResInv;\n"
        "uniform vec2 osgOcean_LowRes;\n"
        "\n"
        "#define NUM_TAPS 4\n"
        "\n"
        "// maximum CoC radius and diameter in pixels\n"
        "const vec2 vMaxCoC = vec2(5.0,10);\n"
        "\n"
        "// scale factor for maximum CoC size on low res. image\n"
        "const float radiusScale = 0.4;\n"
        "\n"
        "// contains poisson-distributed positions on the unit circle\n"
        "vec2 poisson[8];\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "    poisson[0] = vec2( 0.0,       0.0);\n"
        "    poisson[1] = vec2( 0.527837, -0.085868);\n"
        "    poisson[2] = vec2(-0.040088,  0.536087);\n"
        "    poisson[3] = vec2(-0.670445, -0.179949);\n"
        "    poisson[4] = vec2(-0.419418, -0.616039);\n"
        "    poisson[5] = vec2( 0.440453, -0.639399);\n"
        "    poisson[6] = vec2(-0.757088,  0.349334);\n"
        "    poisson[7] = vec2( 0.574619,  0.685879);\n"
        "\n"
        "    float discRadius, discRadiusLow, centerDepth;\n"
        "\n"
        "    // pixel size (1/image resolution) of full resolution image\n"
        "    vec2 pixelSizeHigh = osgOcean_ScreenResInv;\n"
        "\n"
        "    // pixel size of low resolution image\n"
        "    vec2 pixelSizeLow = 4.0 * pixelSizeHigh;\n"
        "\n"
        "    vec4 color = texture2DRect( osgOcean_FullColourMap, gl_TexCoord[0].st );    // fetch center tap\n"
        "    centerDepth = color.a; // save its depth\n"
        "\n"
        "    // convert depth into blur radius in pixels\n"
        "    discRadius = abs(color.a * vMaxCoC.y - vMaxCoC.x);\n"
        "\n"
        "    // compute disc radius on low-res image\n"
        "    discRadiusLow = discRadius * radiusScale;\n"
        "\n"
        "    // reuse color as an accumulator\n"
        "    color = vec4(0.0);\n"
        "\n"
        "    for(int t = 0; t < NUM_TAPS; t++)\n"
        "    {\n"
        "        // fetch low-res tap\n"
        "        vec2 coordLow = gl_TexCoord[1].st + ( osgOcean_LowRes * (pixelSizeLow * poisson[t] * discRadiusLow) );\n"
        "        vec4 tapLow = texture2DRect(osgOcean_BlurMap, coordLow);\n"
        "\n"
        "        // fetch high-res tap\n"
        "        vec2 coordHigh = gl_TexCoord[0].st + ( osgOcean_ScreenRes * (pixelSizeHigh * poisson[t] * discRadius) );\n"
        "        vec4 tapHigh = texture2DRect(osgOcean_FullColourMap, coordHigh);\n"
        "\n"
        "        // put tap blurriness into [0, 1] range\n"
        "        float tapBlur = abs(tapHigh.a * 2.0 - 1.0);\n"
        "\n"
        "        // mix low- and hi-res taps based on tap blurriness\n"
        "        vec4 tap = mix(tapHigh, tapLow, tapBlur);\n"
        "\n"
        "        // apply leaking reduction: lower weight for taps that are\n"
        "        // closer than the center tap and in focus\n"
        "        tap.a = (tap.a >= centerDepth) ? 1.0 : abs(tap.a * 2.0 - 1.0);\n"
        "\n"
        "        // accumulate\n"
        "        color.rgb += tap.rgb * tap.a;\n"
        "        color.a += tap.a;\n"
        "    }\n"
        "\n"
        "    // normalize and return result\n"
        "    gl_FragColor = color / color.a;\n"
        "}\n";
#else
    static const char dof_composite_vertex[]   = "dof_combiner.vert";
    static const char dof_composite_fragment[] = "dof_combiner.frag";
#endif

    osg::Vec2f screenRes( (float)_screenDims.x(), (float)_screenDims.y() );
    osg::Vec2f invScreenRes( 1.f / (float)_screenDims.x(), 1.f / (float)_screenDims.y() );
    osg::Vec2f lowRes( float(_screenDims.x())*0.25f, float(_screenDims.y())*0.25f );

    osg::StateSet* ss = new osg::StateSet;
    ss->setTextureAttributeAndModes( 0, fullscreenTexture, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes( 1, blurTexture, osg::StateAttribute::ON );
    
    ss->setAttributeAndModes( ShaderManager::instance().createProgram("dof_combiner", dof_composite_vertex, dof_composite_fragment, !USE_LOCAL_SHADERS ), osg::StateAttribute::ON );
    
    ss->addUniform( new osg::Uniform( "osgOcean_FullColourMap", 0 ) );
    ss->addUniform( new osg::Uniform( "osgOcean_BlurMap",       1 ) );
    ss->addUniform( new osg::Uniform( "osgOcean_ScreenRes",     screenRes ) );
    ss->addUniform( new osg::Uniform( "osgOcean_ScreenResInv",  invScreenRes ) );
    ss->addUniform( new osg::Uniform( "osgOcean_LowRes",        lowRes ) );

    osg::Geode* combinedDOFQuad = createScreenQuad( _screenDims, osg::Vec2s(1,1) );
    combinedDOFQuad->setStateSet(ss);

    osg::Camera* dofCombineCamera = renderToTexturePass( outputTexture );
    dofCombineCamera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0, _screenDims.y(), 1, 10 );
    dofCombineCamera->addChild( combinedDOFQuad );

    return dofCombineCamera;
}

osg::Camera* OceanScene::dofFinalPass( osg::TextureRectangle* combinedTexture )
{
    _distortionSurface = new DistortionSurface(osg::Vec3f(0,0,-1), osg::Vec2f(_screenDims.x(),_screenDims.y()), combinedTexture);

    osg::Camera* camera = new osg::Camera;
    camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );
    camera->addChild(_distortionSurface);

    return camera;
}

osg::Camera* OceanScene::glarePass(osg::TextureRectangle* streakInput, 
                                              osg::TextureRectangle* steakOutput, 
                                              int pass, 
                                              osg::Vec2f direction )
{
#if USE_LOCAL_SHADERS
    static const char streak_vertex[] =
        "void main(void)\n"
        "{\n"
        "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "    gl_Position = ftransform();\n"
        "}\n";

    static const char streak_fragment[] = 
        "#define NUM_SAMPLES 4\n"
        "\n"
        "uniform sampler2DRect osgOcean_Buffer;\n"
        "uniform vec2 osgOcean_Direction;\n"
        "uniform float osgOcean_Attenuation;\n"
        "uniform float osgOcean_Pass;\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "    vec2 sampleCoord = vec2(0.0);\n"
        "    vec3 cOut = vec3(0.0);\n"
        "    \n"
        "    // sample weight = a^(b*s)\n"
        "    // a = attenuation\n"
        "    // b = 4^(pass-1)\n"
        "    // s = sample number\n"
        "    \n"
        "    vec2 pxSize = vec2(0.5);\n"
        "\n"
        "    float b = pow( float(NUM_SAMPLES), float(osgOcean_Pass));\n"
        "    float sf = 0.0;\n"
        "\n"
        "    for (int s = 0; s < NUM_SAMPLES; s++)\n"
        "    {\n"
        "        sf = float(s);\n"
        "        float weight = pow(osgOcean_Attenuation, b * sf);\n"
        "        sampleCoord = gl_TexCoord[0].st + (osgOcean_Direction * b * vec2(sf) * pxSize);\n"
        "        cOut += clamp(weight,0.0,1.0) * texture2DRect(osgOcean_Buffer, sampleCoord).rgb;\n"
        "    }\n"
        "    \n"
        "    vec3 streak = clamp(cOut, 0.0, 1.0);\n"
        "\n"
        "    gl_FragColor = vec4(streak,1.0);\n"
        "}\n";
#else
    static const char streak_vertex[]   = "streak.vert";
    static const char streak_fragment[] = "streak.frag";
#endif
    osg::Vec2s lowResDims = _screenDims / 4;

    osg::Camera* glarePass = renderToTexturePass( steakOutput );
    glarePass->setClearColor( osg::Vec4f( 0.f, 0.f, 0.f, 0.f) );
    glarePass->setProjectionMatrixAsOrtho( 0, lowResDims.x(), 0.f, lowResDims.y(), 1.0, 500.f );
    glarePass->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    {
        osg::Program* program = ShaderManager::instance().createProgram( "streak_shader", streak_vertex, streak_fragment, !USE_LOCAL_SHADERS );

        osg::Geode* screenQuad = createScreenQuad(lowResDims, lowResDims);
        screenQuad->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        screenQuad->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Buffer", 0) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Pass", float(pass)) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Direction", direction) );
        screenQuad->getStateSet()->addUniform( new osg::Uniform("osgOcean_Attenuation", _glareAttenuation ) );
        screenQuad->getOrCreateStateSet()->setTextureAttributeAndModes(0,streakInput,osg::StateAttribute::ON);
        glarePass->addChild( screenQuad ); 
    }

    return glarePass;
}

osg::Camera* OceanScene::glareCombinerPass(osg::TextureRectangle* fullscreenTexture,
                                                         osg::TextureRectangle* glareTexture1,
                                                         osg::TextureRectangle* glareTexture2,
                                                         osg::TextureRectangle* glareTexture3,
                                                         osg::TextureRectangle* glareTexture4 )
{
#if USE_LOCAL_SHADERS
    static const char glare_composite_vertex[] = 
        "void main(void)\n"
        "{\n"
        "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "    gl_TexCoord[1] = gl_MultiTexCoord0 * vec4(0.25,0.25,1.0,1.0);\n"
        "\n"
        "    gl_Position = ftransform();\n"
        "}\n";

    static const char glare_composite_fragment[] = 
        "uniform sampler2DRect osgOcean_ColorBuffer;\n"
        "uniform sampler2DRect osgOcean_StreakBuffer1;\n"
        "uniform sampler2DRect osgOcean_StreakBuffer2;\n"
        "uniform sampler2DRect osgOcean_StreakBuffer3;\n"
        "uniform sampler2DRect osgOcean_StreakBuffer4;\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "    vec4 fullColor    = texture2DRect(osgOcean_ColorBuffer,   gl_TexCoord[0].st );\n"
        "    vec4 streakColor1 = texture2DRect(osgOcean_StreakBuffer1, gl_TexCoord[1].st );\n"
        "    vec4 streakColor2 = texture2DRect(osgOcean_StreakBuffer2, gl_TexCoord[1].st );\n"
        "    vec4 streakColor3 = texture2DRect(osgOcean_StreakBuffer3, gl_TexCoord[1].st );\n"
        "    vec4 streakColor4 = texture2DRect(osgOcean_StreakBuffer4, gl_TexCoord[1].st );\n"
        "\n"
        "    vec4 streak = streakColor1+streakColor2+streakColor3+streakColor4;\n"
        "    \n"
        "    gl_FragColor = vec4( streak.rgb+fullColor.rgb, 1.0);\n"
        "}\n";
#else
    static const char glare_composite_vertex[]   = "glare_composite.vert";
    static const char glare_composite_fragment[] = "glare_composite.frag";
#endif
    osg::Camera* camera = new osg::Camera;

    camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    camera->setClearColor( osg::Vec4(0.f, 0.f, 0.f, 1.0) );
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT);
    camera->setProjectionMatrixAsOrtho( 0, _screenDims.x(), 0.f, _screenDims.y(), 1.0, 500.f );
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setViewport( 0, 0, _screenDims.x(), _screenDims.y() );

    osg::Geode* quad = createScreenQuad( _screenDims, _screenDims );

    osg::Program* program = ShaderManager::instance().createProgram( "glare_composite", glare_composite_vertex, glare_composite_fragment, !USE_LOCAL_SHADERS );

    osg::StateSet* ss = quad->getOrCreateStateSet();
    ss->setAttributeAndModes(program, osg::StateAttribute::ON);
    ss->setTextureAttributeAndModes(0, fullscreenTexture, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(1, glareTexture1, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(2, glareTexture2, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(3, glareTexture3, osg::StateAttribute::ON );
    ss->setTextureAttributeAndModes(4, glareTexture4, osg::StateAttribute::ON );
    ss->addUniform( new osg::Uniform("osgOcean_ColorBuffer",   0 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer1", 1 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer2", 2 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer3", 3 ) );
    ss->addUniform( new osg::Uniform("osgOcean_StreakBuffer4", 4 ) );

    camera->addChild( quad );

    return camera;
}

osg::Texture2D* OceanScene::createTexture2D( const osg::Vec2s& size, GLint format )
{
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setTextureSize(size.x(), size.y());
    texture->setInternalFormat(format);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP );
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP );
    texture->setDataVariance(osg::Object::DYNAMIC);
    return texture;
}

osg::TextureRectangle* OceanScene::createTextureRectangle( const osg::Vec2s& size, GLint format )
{
    osg::TextureRectangle* texture = new osg::TextureRectangle();
    texture->setTextureSize(size.x(), size.y());
    texture->setInternalFormat(format);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP );
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP );
    texture->setDataVariance(osg::Object::DYNAMIC);
    return texture;
}

osg::Geode* OceanScene::createScreenQuad( const osg::Vec2s& dims, const osg::Vec2s& texSize )
{
    osg::Geode* geode = new osg::Geode;

    osg::Geometry* quad = osg::createTexturedQuadGeometry( 
        osg::Vec3f(0.f,0.f,0.f), 
        osg::Vec3f(dims.x(), 0.f, 0.f), 
        osg::Vec3f( 0.f,dims.y(), 0.0 ),
        (float)texSize.x(),
        (float)texSize.y() );

    geode->addDrawable(quad);

    return geode;
}

osg::Program* OceanScene::createDefaultSceneShader(void)
{
#if USE_LOCAL_SHADERS
    static const char default_scene_vertex[] = 
        "// osgOcean Uniforms\n"
        "// -----------------\n"
        "uniform mat4 osg_ViewMatrixInverse;\n"
        "uniform float osgOcean_WaterHeight;\n"
        "uniform vec3 osgOcean_Eye;\n"
        "uniform vec3 osgOcean_UnderwaterAttenuation;\n"
        "uniform vec4 osgOcean_UnderwaterDiffuse;\n"
        "// -----------------\n"
        "\n"
        "varying vec3 vExtinction;\n"
        "varying vec3 vInScattering;\n"
        "\n"
        "varying vec3 vNormal;\n"
        "varying vec3 vLightDir;\n"
        "varying vec3 vEyeVec;\n"
        "varying float vWorldHeight;\n"
        "\n"
        "void computeScattering( in vec3 eye, in vec3 worldVertex, out vec3 extinction, out vec3 inScattering )\n"
        "{\n"
        "	float viewDist = length(eye-worldVertex);\n"
        "	\n"
        "	float depth = max(osgOcean_WaterHeight-worldVertex.z, 0.0);\n"
        "	\n"
        "	extinction = exp(-osgOcean_UnderwaterAttenuation*viewDist*2.0);\n"
        "\n"
        "	// Need to compute accurate kd constant.\n"
        "	// const vec3 kd = vec3(0.001, 0.001, 0.001);\n"
        "	inScattering = osgOcean_UnderwaterDiffuse.rgb * (1.0-extinction*exp(-depth*vec3(0.001)));\n"
        "}\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "	gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "	gl_Position = ftransform();\n"
        "	gl_FogFragCoord = gl_Position.z;\n"
        "	gl_ClipVertex = gl_ModelViewMatrix * gl_Vertex; // for reflections\n"
        "\n"
        "	vNormal = gl_NormalMatrix * gl_Normal;\n"
        "	vLightDir = gl_LightSource[osgOcean_LightID].position.xyz;\n"
        "	vEyeVec = -vec3(gl_ModelViewMatrix*gl_Vertex);\n"
        "\n"
        "	vec4 worldVertex = (osg_ViewMatrixInverse*gl_ModelViewMatrix) * gl_Vertex;\n"
        "\n"
        "	computeScattering( osgOcean_Eye, worldVertex.xyz, vExtinction, vInScattering);\n"
        "\n"
        "	vWorldHeight = worldVertex.z;\n"
        "}\n";
    static const char default_scene_fragment[] = 
        "// osgOcean Uniforms\n"
        "// -----------------\n"
        "uniform float osgOcean_DOF_Near;\n"
        "uniform float osgOcean_DOF_Focus;\n"
        "uniform float osgOcean_DOF_Far;\n"
        "uniform float osgOcean_DOF_Clamp;\n"
        "\n"
        "uniform float osgOcean_UnderwaterFogDensity;\n"
        "uniform float osgOcean_AboveWaterFogDensity;\n"
        "uniform vec4  osgOcean_UnderwaterFogColor;\n"
        "uniform vec4  osgOcean_AboveWaterFogColor;\n"
        "\n"
        "uniform float osgOcean_WaterHeight;\n"
        "\n"
        "uniform bool osgOcean_EnableGlare;\n"
        "uniform bool osgOcean_EnableDOF;\n"
        "uniform bool osgOcean_EyeUnderwater;\n"
        "// -------------------\n"
        "\n"
        "uniform sampler2D uTextureMap;\n"
        "\n"
        "varying vec3 vExtinction;\n"
        "varying vec3 vInScattering;\n"
        "varying vec3 vNormal;\n"
        "varying vec3 vLightDir;\n"
        "varying vec3 vEyeVec;\n"
        "\n"
        "varying float vWorldHeight;\n"
        "\n"
        "float computeDepthBlur(float depth, float focus, float near, float far, float clampval )\n"
        "{\n"
        "   float f;\n"
        "   if (depth < focus){\n"
        "      f = (depth - focus)/(focus - near);\n"
        "   }\n"
        "   else{\n"
        "      f = (depth - focus)/(far - focus);\n"
        "      f = clamp(f, 0.0, clampval);\n"
        "   }\n"
        "   return f * 0.5 + 0.5;\n"
        "}\n"
        "\n"
        "vec4 lighting( vec4 colormap )\n"
        "{\n"
        "	vec4 final_color = gl_LightSource[osgOcean_LightID].ambient * colormap;\n"
        "\n"
        "	vec3 N = normalize(vNormal);\n"
        "	vec3 L = normalize(vLightDir);\n"
        "\n"
        "	float lambertTerm = dot(N,L);\n"
        "\n"
        "	if(lambertTerm > 0.0)\n"
        "	{\n"
        "		final_color += gl_LightSource[osgOcean_LightID].diffuse * lambertTerm * colormap;\n"
        "\n"
        "		vec3 E = normalize(vEyeVec);\n"
        "		vec3 R = reflect(-L, N);\n"
        "\n"
        "		float specular = pow( max(dot(R, E), 0.0), 2.0 );\n"
        "\n"
        "		final_color += gl_LightSource[osgOcean_LightID].specular * specular;\n"
        "	}\n"
        "\n"
        "	return final_color;\n"
        "}\n"
        "\n"
        "float computeFogFactor( float density, float fogCoord )\n"
        "{\n"
        "	return exp2(density * fogCoord * fogCoord );\n"
        "}\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "	vec4 textureColor = texture2D( uTextureMap, gl_TexCoord[0].st );\n"
        "\n"
        "	vec4 final_color;\n"
        "\n"
        "	float alpha;\n"
        "\n"
        "    // Underwater\n"
        "	// +2 tweak here as waves peak above average wave height,\n"
        "	// and surface fog becomes visible.\n"
        "	if(osgOcean_EyeUnderwater && vWorldHeight < osgOcean_WaterHeight+2.0 )\n"
        "	{\n"
        "		final_color = lighting( textureColor );\n"
        "\n"
        "        // mix in underwater light\n"
        "		final_color.rgb = final_color.rgb * vExtinction + vInScattering;\n"
        "\n"
        "		float fogFactor = computeFogFactor( osgOcean_UnderwaterFogDensity, gl_FogFragCoord );\n"
        "\n"
        "		final_color = mix( osgOcean_UnderwaterFogColor, final_color, fogFactor );\n"
        "\n"
        "		if(osgOcean_EnableDOF)\n"
        "        {\n"
        "			final_color.a = computeDepthBlur(gl_FogFragCoord, osgOcean_DOF_Focus, osgOcean_DOF_Near, osgOcean_DOF_Far, osgOcean_DOF_Clamp);\n"
        "        }\n"
        "	}\n"
        "    // Above water\n"
        "	else\n"
        "	{\n"
        "        final_color = lighting( textureColor );\n"
        "\n"
        "		float fogFactor = computeFogFactor( osgOcean_AboveWaterFogDensity, gl_FogFragCoord );\n"
        "		final_color = mix( osgOcean_AboveWaterFogColor, final_color, fogFactor );\n"
        "\n"
        "		if(osgOcean_EnableGlare)\n"
        "        {\n"
        "			final_color.a = 0.0;\n"
        "        }\n"
        "	}\n"
        "\n"
        "	gl_FragColor = final_color;\n"
        "}\n";
#else
    static const char default_scene_vertex[]   = "default_scene.vert";
    static const char default_scene_fragment[] = "default_scene.frag";
#endif
    return ShaderManager::instance().createProgram("scene_shader", default_scene_vertex, default_scene_fragment, !USE_LOCAL_SHADERS );
}

void OceanScene::addResourcePaths(void)
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
//     Callback implementations
// -------------------------------

OceanScene::CameraCullCallback::CameraCullCallback(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

void OceanScene::CameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
    _oceanScene->osg::Group::traverse(*nv);
}

OceanScene::PrerenderCameraCullCallback::PrerenderCameraCullCallback(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

void OceanScene::PrerenderCameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*> (nv);
    
    if(cv)
        _oceanScene->cull(*cv);
}


OceanScene::EventHandler::EventHandler(OceanScene* oceanScene):
_oceanScene(oceanScene)
{
}

bool OceanScene::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    if (ea.getHandled()) return false;

    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYUP):
        {
            // Reflections
            if (ea.getKey() == 'r')
            {
                _oceanScene->enableReflections(!_oceanScene->areReflectionsEnabled());
                osg::notify(osg::NOTICE) << "Reflections " << (_oceanScene->areReflectionsEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Refractions
            if (ea.getKey() == 'R')
            {
                _oceanScene->enableRefractions(!_oceanScene->areRefractionsEnabled());
                osg::notify(osg::NOTICE) << "Refractions " << (_oceanScene->areRefractionsEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // DOF
            if (ea.getKey() == 'o')
            {
                _oceanScene->enableUnderwaterDOF(!_oceanScene->isUnderwaterDOFEnabled());
                osg::notify(osg::NOTICE) << "Depth of field " << (_oceanScene->isUnderwaterDOFEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Glare
            if (ea.getKey() == 'g')
            {
                _oceanScene->enableGlare(!_oceanScene->isGlareEnabled());
                osg::notify(osg::NOTICE) << "Glare " << (_oceanScene->isGlareEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // God rays
            if (ea.getKey() == 'G')
            {
                _oceanScene->enableGodRays(!_oceanScene->areGodRaysEnabled());
                osg::notify(osg::NOTICE) << "God rays " << (_oceanScene->areGodRaysEnabled()? "enabled" : "disabled") << std::endl;
                return true;
            }
            // Silt
            if (ea.getKey() == 't')
            {
                _oceanScene->enableSilt(!_oceanScene->isSiltEnabled());
                osg::notify(osg::NOTICE) << "Silt " << (_oceanScene->isSiltEnabled()? "enabled" : "disabled") << std::endl;
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
void OceanScene::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("r","Toggle reflections (above water)");
    usage.addKeyboardMouseBinding("R","Toggle refractions (underwater)");
    usage.addKeyboardMouseBinding("o","Toggle Depth of Field (DOF) (underwater)");
    usage.addKeyboardMouseBinding("g","Toggle glare (above water)");
    usage.addKeyboardMouseBinding("G","Toggle God rays (underwater)");
    usage.addKeyboardMouseBinding("t","Toggle silt (underwater)");
}
