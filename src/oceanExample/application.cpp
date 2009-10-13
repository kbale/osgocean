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

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osg/Notify>
#include <osg/TextureCubeMap>
#include <osgDB/ReadFile>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osgText/Text>
#include <osg/CullFace>
#include <osg/Fog>
#include <osgText/Font>
#include <osg/Switch>
#include <osg/Texture3D>

#include <string>
#include <vector>

#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>

#include "SkyDome.h"

#define USE_CUSTOM_SHADER

// ----------------------------------------------------
//                  Text HUD Class
// ----------------------------------------------------

class TextHUD : public osg::Referenced
{
private:
    osg::ref_ptr< osg::Camera > _camera;
    osg::ref_ptr< osgText::Text > _modeText;    
    osg::ref_ptr< osgText::Text > _cameraModeText;    

public:
    TextHUD( void ){
        _camera = createCamera();
        _camera->addChild( createText() );
    }

    osg::Camera* createCamera( void )
    {
        osg::Camera* camera=new osg::Camera;

        camera->setViewport(0,0,1024,768);
        camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
        camera->setProjectionMatrixAsOrtho2D(0,1024,0,768);
        camera->setRenderOrder(osg::Camera::POST_RENDER);
        camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
        camera->setClearMask(GL_DEPTH_BUFFER_BIT);

        return camera;
    }

    osg::Node* createText( void )
    {
        osg::Geode* textGeode = new osg::Geode;

        osgText::Text* title = new osgText::Text;
        title->setFont("fonts/arial.ttf");
        title->setCharacterSize(14);
        title->setLineSpacing(0.4f);
        title->setText("osgOcean 1.0\nPress 'h' for options");
        textGeode->addDrawable( title );

        _modeText = new osgText::Text;
        _modeText->setFont("fonts/arial.ttf");
        _modeText->setCharacterSize(14);
        _modeText->setPosition( osg::Vec3f(0.f, -40.f, 0.f ) );
        _modeText->setDataVariance(osg::Object::DYNAMIC);
        textGeode->addDrawable( _modeText.get() );

        _cameraModeText = new osgText::Text;
        _cameraModeText->setFont("fonts/arial.ttf");
        _cameraModeText->setCharacterSize(14);
        _cameraModeText->setPosition( osg::Vec3f(0.f, -60.f, 0.f ) );
        _cameraModeText->setDataVariance(osg::Object::DYNAMIC);
        textGeode->addDrawable( _cameraModeText.get() );

        osg::PositionAttitudeTransform* titlePAT = new osg::PositionAttitudeTransform;
        titlePAT->setPosition( osg::Vec3f( 10, 70, 0.f ) );
        titlePAT->addChild(textGeode);

        return titlePAT;
    }

    void setSceneText( const std::string& preset )
    {
        _modeText->setText( "Preset: " + preset );
    }

    void setCameraText(const std::string& mode )
    {
        _cameraModeText->setText( "Camera: " + mode );
    }

    osg::Camera* getHudCamera(void)
    {
        return _camera.get();
    }
};

// ----------------------------------------------------
//               Camera Track Callback
// ----------------------------------------------------

class CameraTrackCallback: public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            osg::Vec3f centre,up;
            cv->getCurrentCamera()->getViewMatrixAsLookAt(_eye,centre,up);
        }
        else if(nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
            osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*>(node);
            pat->setPosition( osg::Vec3f(_eye.x(), _eye.y(), pat->getPosition().z() ) );
        }

        traverse(node, nv); 
    }

    osg::Vec3f _eye;
};


class BoatPositionCallback : public osg::NodeCallback
{
public: 
    BoatPositionCallback(osgOcean::OceanScene* oceanScene)
        : _oceanScene(oceanScene) {}

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if(nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR ){
            osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);
            if (!mt || !_oceanScene.valid()) return;

            osg::Matrix mat = osg::computeLocalToWorld(nv->getNodePath());
            osg::Vec3d pos = mat.getTrans();

            osg::Vec3f normal;
            // Get the ocean surface height at the object's position, note
            // that this only considers one point on the object (the object's
            // geometric center) and not the whole object.
            float height = _oceanScene->getOceanSurfaceHeightAt(pos.x(), pos.y(), &normal);

            mat.makeTranslate(osg::Vec3f(pos.x(), pos.y(), height));

            osg::Matrix rot;
            rot.makeIdentity();
            rot.makeRotate( normal.x(), osg::Vec3f(1.0f, 0.0f, 0.0f), 
                            normal.y(), osg::Vec3f(0.0f, 1.0f, 0.0f),
                            (1.0f-normal.z()), osg::Vec3f(0.0f, 0.0f, 1.0f));

            mat = rot*mat;
            mt->setMatrix(mat);
        }

        traverse(node, nv); 
    }

    osg::observer_ptr<osgOcean::OceanScene> _oceanScene;
};



// ----------------------------------------------------
//                  Scoped timer
// ----------------------------------------------------

class ScopedTimer
{
public:
    ScopedTimer(const std::string& description, 
                std::ostream& output_stream = std::cout, 
                bool endline_after_time = true)
        : _output_stream(output_stream)
        , _start()
        , _endline_after_time(endline_after_time)
    {
        _output_stream << description << std::flush;
        _start = osg::Timer::instance()->tick();
    }

    ~ScopedTimer()
    {
        osg::Timer_t end = osg::Timer::instance()->tick();
        _output_stream << osg::Timer::instance()->delta_s(_start, end) << "s";
        if (_endline_after_time) _output_stream << std::endl;
        else                     _output_stream << std::flush;
    }

private:
    std::ostream& _output_stream;
    osg::Timer_t _start;
    bool _endline_after_time;
};

// ----------------------------------------------------
//                  Scene Model
// ----------------------------------------------------

class SceneModel : public osg::Referenced
{
public:
    enum SCENE_TYPE{ CLEAR, DUSK, CLOUDY };

private:
    SCENE_TYPE _sceneType;

    osg::ref_ptr<osgText::Text> _modeText;
    osg::ref_ptr<osg::Group> _scene;

    osg::ref_ptr<osgOcean::OceanScene> _oceanScene;
    osg::ref_ptr<osgOcean::FFTOceanSurface> _oceanSurface;
    osg::ref_ptr<osg::TextureCubeMap> _cubemap;
    osg::ref_ptr<SkyDome> _skyDome;
        
    std::vector<std::string> _cubemapDirs;
    std::vector<osg::Vec4f>  _lightColors;
    std::vector<osg::Vec4f>  _fogColors;
    std::vector<osg::Vec3f>  _underwaterAttenuations;
    std::vector<osg::Vec4f>  _underwaterDiffuse;
        
    osg::ref_ptr<osg::Light> _light;

    std::vector<osg::Vec3f>  _sunPositions;
    std::vector<osg::Vec4f>  _sunDiffuse;
    std::vector<osg::Vec4f>  _waterFogColors;

    osg::ref_ptr<osg::Switch> _islandSwitch;

public:
    SceneModel( const osg::Vec2f& windDirection = osg::Vec2f(1.0f,1.0f),
                float windSpeed = 12.f,
                float depth = 10000.f,
                float reflectionDamping = 0.35f,
                float scale = 1e-8,
                bool  isChoppy = true,
                float choppyFactor = -2.5f,
                float crestFoamHeight = 2.2f ):
        _sceneType(CLEAR)
    {
        _cubemapDirs.push_back( "sky_clear" );
        _cubemapDirs.push_back( "sky_dusk" );
        _cubemapDirs.push_back( "sky_fair_cloudy" );

        _fogColors.push_back( intColor( 199,226,255 ) );
        _fogColors.push_back( intColor( 244,228,179 ) );
        _fogColors.push_back( intColor( 172,224,251 ) );

        _waterFogColors.push_back( intColor(27,57,109) );
        _waterFogColors.push_back( intColor(44,69,106 ) );
        _waterFogColors.push_back( intColor(84,135,172 ) );

        _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
        _underwaterAttenuations.push_back( osg::Vec3f(0.015f, 0.0075f, 0.005f) );
        _underwaterAttenuations.push_back( osg::Vec3f(0.008f, 0.003f, 0.002f) );

        _underwaterDiffuse.push_back( intColor(27,57,109) );
        _underwaterDiffuse.push_back( intColor(44,69,106) );
        _underwaterDiffuse.push_back( intColor(84,135,172) );

        _lightColors.push_back( intColor( 105,138,174 ) );
        _lightColors.push_back( intColor( 105,138,174 ) );
        _lightColors.push_back( intColor( 105,138,174 ) );

        _sunPositions.push_back( osg::Vec3f(326.573, 1212.99 ,1275.19) );
        _sunPositions.push_back( osg::Vec3f(520.f, 1900.f, 550.f ) );
        _sunPositions.push_back( osg::Vec3f(-1056.89f, -771.886f, 1221.18f ) );

        _sunDiffuse.push_back( intColor( 191, 191, 191 ) );
        _sunDiffuse.push_back( intColor( 251, 251, 161 ) );
        _sunDiffuse.push_back( intColor( 191, 191, 191 ) );

        build(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight);
    }

    void build( const osg::Vec2f& windDirection,
                float windSpeed,
                float depth,
                float reflectionDamping,
                float waveScale,
                bool  isChoppy,
                float choppyFactor,
                float crestFoamHeight )
    {
        {
            ScopedTimer buildSceneTimer("Building scene... \n", osg::notify(osg::NOTICE));

            _scene = new osg::Group; 

            {
                ScopedTimer cubemapTimer("  . Loading cubemaps: ", osg::notify(osg::NOTICE));
                _cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
            }

            // Set up surface
            {
                ScopedTimer oceanSurfaceTimer("  . Generating ocean surface: ", osg::notify(osg::NOTICE));
                _oceanSurface = new osgOcean::FFTOceanSurface( 64, 256, 17, 
                    windDirection, windSpeed, depth, reflectionDamping, waveScale, isChoppy, choppyFactor, 10.f, 256 );  

                _oceanSurface->setEnvironmentMap( _cubemap.get() );
                _oceanSurface->setFoamBottomHeight( 2.2f );
                _oceanSurface->setFoamTopHeight( 3.0f );
                _oceanSurface->enableCrestFoam( true );
                _oceanSurface->setLightColor( _lightColors[_sceneType] );
                _oceanSurface->enableEndlessOcean(true);
            }

            // Set up ocean scene, add surface
            {
                ScopedTimer oceanSceneTimer("  . Creating ocean scene: ", osg::notify(osg::NOTICE));
                osg::Vec3f sunDir = -_sunPositions[_sceneType];
                sunDir.normalize();
                
                _oceanScene = new osgOcean::OceanScene( _oceanSurface.get() );
                _oceanScene->setLightID(0);
                _oceanScene->enableReflections(true);
                _oceanScene->enableRefractions(true);
                
                _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
                _oceanScene->setUnderwaterFog(0.002f,  _waterFogColors[_sceneType] );
                _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
                _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );

                _oceanScene->setSunDirection( sunDir );
                _oceanScene->enableGodRays(true);
                _oceanScene->enableSilt(true);
                _oceanScene->enableUnderwaterDOF(true);
                _oceanScene->enableDistortion(true);
                _oceanScene->enableGlare(true);
                _oceanScene->setGlareAttenuation(0.8f);

                // create sky dome and add to ocean scene
                // set masks so it appears in reflected scene and normal scene
                _skyDome = new SkyDome( 1900.f, 16, 16, _cubemap.get() );
                _skyDome->setNodeMask( _oceanScene->getReflectedSceneMask() | _oceanScene->getNormalSceneMask() );

                // add a pat to track the camera
                osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform;
                pat->setDataVariance( osg::Object::DYNAMIC );
                pat->setPosition( osg::Vec3f(0.f, 0.f, 0.f) );
                pat->setUpdateCallback( new CameraTrackCallback );
                pat->setCullCallback( new CameraTrackCallback );
                
                pat->addChild( _skyDome.get() );

                _oceanScene->addChild( pat );

                {
                    // Create and add fake texture for use with nodes without any texture
                    // since the OceanScene default scene shader assumes that texture unit 
                    // 0 is used as a base texture map.
                    osg::Image * image = new osg::Image;
                    image->allocateImage( 1, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE );
                    *(osg::Vec4ub*)image->data() = osg::Vec4ub( 0xFF, 0xFF, 0xFF, 0xFF );
                    
                    osg::Texture2D* fakeTex = new osg::Texture2D( image );
                    fakeTex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
                    fakeTex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
                    fakeTex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::NEAREST);
                    fakeTex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::NEAREST);
                    
                    osg::StateSet* stateset = _oceanScene->getOrCreateStateSet();
                    stateset->setTextureAttribute(0,fakeTex,osg::StateAttribute::ON);
                    stateset->setTextureMode(0,GL_TEXTURE_1D,osg::StateAttribute::OFF);
                    stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::ON);
                    stateset->setTextureMode(0,GL_TEXTURE_3D,osg::StateAttribute::OFF);
                }

            }

            {
                ScopedTimer islandsTimer("  . Loading islands: ", osg::notify(osg::NOTICE));
                osg::ref_ptr<osg::Node> islandModel = loadIslands();

                if( islandModel.valid() )
                {
                    _islandSwitch = new osg::Switch;
                    _islandSwitch->addChild( islandModel.get(), true );
                    _islandSwitch->setNodeMask( _oceanScene->getNormalSceneMask() | _oceanScene->getReflectedSceneMask() | _oceanScene->getRefractedSceneMask() );
                    _oceanScene->addChild( _islandSwitch.get() );
                }
            }

            {
                ScopedTimer lightingTimer("  . Setting up lighting: ", osg::notify(osg::NOTICE));
                osg::LightSource* lightSource = new osg::LightSource;
                lightSource->setLocalStateSetModes();

                _light = lightSource->getLight();
                _light->setLightNum(0);
                _light->setAmbient( osg::Vec4d(0.3f, 0.3f, 0.3f, 1.0f ));
                _light->setDiffuse( _sunDiffuse[_sceneType] );
                _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
                _light->setPosition( osg::Vec4f(_sunPositions[_sceneType], 1.f) ); // point light

                _scene->addChild( lightSource );
                _scene->addChild( _oceanScene.get() );
                //_scene->addChild( sunDebug(_sunPositions[CLOUDY]) );
            }

            osg::notify(osg::NOTICE) << "complete.\nTime Taken: ";
        }
    }

    osgOcean::OceanTechnique* getOceanSurface( void )
    {
        return _oceanSurface.get();
    }

    osg::Group* getScene(void){
        return _scene.get();
    }

    osgOcean::OceanScene* getOceanScene()
    {
        return _oceanScene.get();
    }

    void changeScene( SCENE_TYPE type )
    {
        _sceneType = type;

        _cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
        _skyDome->setCubeMap( _cubemap.get() );
        _oceanSurface->setEnvironmentMap( _cubemap.get() );
        _oceanSurface->setLightColor( _lightColors[type] );

        _oceanScene->setAboveWaterFog(0.0012f, _fogColors[_sceneType] );
        _oceanScene->setUnderwaterFog(0.002f,  _waterFogColors[_sceneType] );
        _oceanScene->setUnderwaterDiffuse( _underwaterDiffuse[_sceneType] );
        _oceanScene->setUnderwaterAttenuation( _underwaterAttenuations[_sceneType] );
        
        osg::Vec3f sunDir = -_sunPositions[_sceneType];
        sunDir.normalize();

        _oceanScene->setSunDirection( sunDir );

        _light->setPosition( osg::Vec4f(_sunPositions[_sceneType],1.f) );
        _light->setDiffuse( _sunDiffuse[_sceneType] ) ;

        if(_islandSwitch.valid() )
        {
            if(_sceneType == CLEAR || _sceneType == CLOUDY)
                _islandSwitch->setAllChildrenOn();
            else
                _islandSwitch->setAllChildrenOff();
        }
    }

    // Load the islands model
    // Here we attach a custom shader to the model.
    // This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
    // The custom shader is needed to add multi-texturing and bump mapping to the terrain.
    osg::Node* loadIslands(void)
    {
        osgDB::Registry::instance()->getDataFilePathList().push_back("resources/island");
        const std::string filename = "islands.ive";
        osg::ref_ptr<osg::Node> island = osgDB::readNodeFile(filename);

        if(!island){
            osg::notify(osg::WARN) << "Could not find: " << filename << std::endl;
            return NULL;
        }

#ifdef USE_CUSTOM_SHADER
        static const char terrain_vertex[]   = "terrain.vert";
        static const char terrain_fragment[] = "terrain.frag";

        osg::Program* program = osgOcean::ShaderManager::instance().createProgram("terrain", terrain_vertex, terrain_fragment, true);
        program->addBindAttribLocation("aTangent", 6);
#endif
        island->setNodeMask( _oceanScene->getNormalSceneMask() | _oceanScene->getReflectedSceneMask() | _oceanScene->getRefractedSceneMask() );
        island->getStateSet()->addUniform( new osg::Uniform( "uTextureMap", 0 ) );

#ifdef USE_CUSTOM_SHADER
        island->getOrCreateStateSet()->setAttributeAndModes(program,osg::StateAttribute::ON);
        island->getStateSet()->addUniform( new osg::Uniform( "uOverlayMap", 1 ) );
        island->getStateSet()->addUniform( new osg::Uniform( "uNormalMap",  2 ) );
#endif
        osg::PositionAttitudeTransform* islandpat = new osg::PositionAttitudeTransform;
        islandpat->setPosition(osg::Vec3f( -island->getBound().center()+osg::Vec3f(0.0, 0.0, -15.f) ) );
        islandpat->setScale( osg::Vec3f(4.f, 4.f, 3.f ) );
        islandpat->addChild(island.get());

        return islandpat;
    }

    osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir )
    {
        enum {POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z};

        std::string filenames[6];

        filenames[POS_X] = "resources/textures/" + dir + "/east.png";
        filenames[NEG_X] = "resources/textures/" + dir + "/west.png";
        filenames[POS_Z] = "resources/textures/" + dir + "/north.png";
        filenames[NEG_Z] = "resources/textures/" + dir + "/south.png";
        filenames[POS_Y] = "resources/textures/" + dir + "/down.png";
        filenames[NEG_Y] = "resources/textures/" + dir + "/up.png";

        osg::ref_ptr<osg::TextureCubeMap> cubeMap = new osg::TextureCubeMap;
        cubeMap->setInternalFormat(GL_RGBA);

        cubeMap->setFilter( osg::Texture::MIN_FILTER,    osg::Texture::LINEAR_MIPMAP_LINEAR);
        cubeMap->setFilter( osg::Texture::MAG_FILTER,    osg::Texture::LINEAR);
        cubeMap->setWrap  ( osg::Texture::WRAP_S,        osg::Texture::CLAMP_TO_EDGE);
        cubeMap->setWrap  ( osg::Texture::WRAP_T,        osg::Texture::CLAMP_TO_EDGE);

        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, osgDB::readImageFile( filenames[NEG_X] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, osgDB::readImageFile( filenames[POS_X] ) );
        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, osgDB::readImageFile( filenames[NEG_Y] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, osgDB::readImageFile( filenames[POS_Y] ) );
        cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, osgDB::readImageFile( filenames[NEG_Z] ) );
        cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, osgDB::readImageFile( filenames[POS_Z] ) );

        return cubeMap;
    }

    osg::Geode* sunDebug( const osg::Vec3f& position )
    {
        osg::ShapeDrawable* sphereDraw = new osg::ShapeDrawable( new osg::Sphere( position, 15.f ) );
        sphereDraw->setColor(osg::Vec4f(1.f,0.f,0.f,1.f));
        
        osg::Geode* sphereGeode = new osg::Geode;
        sphereGeode->addDrawable( sphereDraw );
        
        return sphereGeode;
    }

    osg::Vec4f intColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255 )
    {
        float div = 1.f/255.f;
        return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
    }

    osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler()
    {
        return _oceanScene->getEventHandler();
    }
};

// ----------------------------------------------------
//                   Event Handler
// ----------------------------------------------------

class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
    osg::ref_ptr<SceneModel> _scene;
    osg::ref_ptr<TextHUD> _textHUD;
    osgViewer::Viewer& _viewer;
    
    enum CameraMode
    {
        FIXED,
        FLIGHT,
        TRACKBALL
    };

    CameraMode _currentCameraMode;

public:
    SceneEventHandler( SceneModel* scene, TextHUD* textHUD, osgViewer::Viewer& viewer ):
        _scene(scene),
        _textHUD(textHUD),
        _viewer(viewer),
        _currentCameraMode(FIXED)
    {
        _textHUD->setSceneText("Clear");
        _textHUD->setCameraText("FIXED");

        osg::Vec3f eye(0.f,0.f,20.f);
        osg::Vec3f centre = eye+osg::Vec3f(0.f,1.f,0.f);
        osg::Vec3f up(0.f, 0.f, 1.f);

        _viewer.getCamera()->setViewMatrixAsLookAt( eye, centre, up    );
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
        case(osgGA::GUIEventAdapter::KEYUP):
            {
                if(ea.getKey() == '1')
                {
                    _scene->changeScene( SceneModel::CLEAR );
                    _textHUD->setSceneText( "Clear Blue Sky" );
                    return false;
                }
                else if(ea.getKey() == '2')
                {
                    _scene->changeScene( SceneModel::DUSK );
                    _textHUD->setSceneText( "Dusk" );
                    return false;
                }
                else if(ea.getKey() == '3' )
                {
                    _scene->changeScene( SceneModel::CLOUDY );
                    _textHUD->setSceneText( "Pacific Cloudy" );
                    return false;
                }
                else if(ea.getKey() == 'C' || ea.getKey() == 'c' )
                {
                    if (_currentCameraMode == FIXED)
                    {
                        _currentCameraMode = FLIGHT;
                        osgGA::FlightManipulator* flight = new osgGA::FlightManipulator;
                        flight->setHomePosition( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
                        _viewer.setCameraManipulator( flight );
                        _textHUD->setCameraText("FLIGHT");
                    }
                    else if (_currentCameraMode == FLIGHT)
                    {
                        _currentCameraMode = TRACKBALL;
                        osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
                        tb->setHomePosition( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,20.f,20.f), osg::Vec3f(0,0,1) );
                        _viewer.setCameraManipulator( tb );
                        _textHUD->setCameraText("TRACKBALL");
                    }
                    else if (_currentCameraMode == TRACKBALL)
                    {
                        _currentCameraMode = FIXED;
                        _viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
                        _viewer.setCameraManipulator(NULL);
                        _textHUD->setCameraText("FIXED");
                    }
                }
            }
        default:
            return false;
        }
    }

    void getUsage(osg::ApplicationUsage& usage) const
    {
        usage.addKeyboardMouseBinding("c","Camera type (cycle through Fixed, Flight, Trackball)");
        usage.addKeyboardMouseBinding("1","Select scene \"Clear Blue Sky\"");
        usage.addKeyboardMouseBinding("2","Select scene \"Dusk\"");
        usage.addKeyboardMouseBinding("3","Select scene \"Pacific Cloudy\"");
    }

};

int main(int argc, char *argv[])
{
    osg::ArgumentParser arguments(&argc,argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is an example of osgOcean.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] ...");
    arguments.getApplicationUsage()->addCommandLineOption("--windx <x>","Wind X direction. Default 1.1");
    arguments.getApplicationUsage()->addCommandLineOption("--windy <y>","Wind Y direction. Default 1.1");
    arguments.getApplicationUsage()->addCommandLineOption("--windSpeed <speed>","Wind speed. Default: 12");
    arguments.getApplicationUsage()->addCommandLineOption("--depth <depth>","Depth. Default: 10000");
    arguments.getApplicationUsage()->addCommandLineOption("--isNotChoppy","Set the waves not choppy (by default they are).");
    arguments.getApplicationUsage()->addCommandLineOption("--choppyFactor <factor>","How choppy the waves are. Default: 2.5");
    arguments.getApplicationUsage()->addCommandLineOption("--crestFoamHeight <height>","How high the waves need to be before foam forms on the crest. Default: 2.2 ");
    arguments.getApplicationUsage()->addCommandLineOption("--oceanSurfaceHeight <z>","Z position of the ocean surface in world coordinates. Default: 0.0");
    arguments.getApplicationUsage()->addCommandLineOption("--testCollision","Test ocean surface collision detection by making a boat float on its surface.");

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

    float windx = 1.1f, windy = 1.1f;
    while (arguments.read("--windx", windx));
    while (arguments.read("--windy", windy));
    osg::Vec2f windDirection(windx, windy);

    float windSpeed = 12.f;
    while (arguments.read("--windSpeed", windSpeed));

    float depth = 1000.f;
    while (arguments.read("--depth", depth));

    float reflectionDamping = 0.35f;
    while (arguments.read("--reflectionDamping", reflectionDamping));

    float scale = 1e-8;
    while (arguments.read("--waveScale", scale ) );

    bool isChoppy = true;
    while (arguments.read("--isNotChoppy")) isChoppy = false;

    float choppyFactor = 2.5f;
    while (arguments.read("--choppyFactor", choppyFactor));
    choppyFactor = -choppyFactor;

    float crestFoamHeight = 2.2f;
    while (arguments.read("--crestFoamHeight", crestFoamHeight));

    double oceanSurfaceHeight = 0.0f;
    while (arguments.read("--oceanSurfaceHeight", oceanSurfaceHeight));

    bool testCollision = false;
    if (arguments.read("--testCollision")) testCollision = true;

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }

    osgViewer::Viewer viewer;

    viewer.setUpViewInWindow( 150,150,1024,768, 0 );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    osg::ref_ptr<TextHUD> hud = new TextHUD;

    osg::ref_ptr<SceneModel> scene = new SceneModel(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight);
    scene->getOceanScene()->setOceanSurfaceHeight(oceanSurfaceHeight);
    viewer.addEventHandler(scene->getOceanSceneEventHandler());
    viewer.addEventHandler(scene->getOceanSurface()->getEventHandler());

    viewer.addEventHandler( new SceneEventHandler(scene.get(), hud.get(), viewer ) );
    viewer.addEventHandler( new osgViewer::HelpHandler );

    osg::Group* root = new osg::Group;
    root->addChild( scene->getScene() );
    root->addChild( hud->getHudCamera() );

    if (loadedModel.valid())
    {
        loadedModel->setNodeMask( scene->getOceanScene()->getNormalSceneMask() | 
                                  scene->getOceanScene()->getReflectedSceneMask() | 
                                  scene->getOceanScene()->getRefractedSceneMask() );
        scene->getOceanScene()->addChild(loadedModel.get());
    }

    if (testCollision)
    {
        osg::ref_ptr<osg::Node> boat = osgDB::readNodeFile("resources/boat.3ds");
        boat->setNodeMask( scene->getOceanScene()->getNormalSceneMask() | 
                           scene->getOceanScene()->getReflectedSceneMask() | 
                           scene->getOceanScene()->getRefractedSceneMask() );

        osg::ref_ptr<osg::MatrixTransform> boatTransform = new osg::MatrixTransform;
        boatTransform->addChild(boat.get());
        boatTransform->setMatrix(osg::Matrix::translate(osg::Vec3f(0.0f, 160.0f,0.0f)));
        boatTransform->setUpdateCallback( new BoatPositionCallback(scene->getOceanScene()) );

        scene->getOceanScene()->addChild(boatTransform.get());   
    }

    viewer.setSceneData( root );

    viewer.realize();

    while(!viewer.done())
    {
        viewer.frame();    
    }

    return 0;
}
