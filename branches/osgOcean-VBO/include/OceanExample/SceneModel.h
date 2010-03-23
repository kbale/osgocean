#pragma once
#include <osg/Switch>
#include <osg/TextureCubeMap>

#include <osgText/Text>

#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/FFTOceanSurfaceVBO>

#include "SkyDome.h"

class SceneModel : public osg::Referenced
{
public:
    enum SCENE_TYPE{ CLEAR, DUSK, CLOUDY };

private:
    SCENE_TYPE _sceneType;
    bool _useVBO;

    osg::ref_ptr<osgText::Text> _modeText;
    osg::ref_ptr<osg::Group> _scene;

    osg::ref_ptr<osgOcean::OceanScene> _oceanScene;
    osg::ref_ptr<osgOcean::OceanTechnique> _oceanSurface;
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
    SceneModel( 
        const osg::Vec2f& windDirection = osg::Vec2f(1.0f,1.0f),
        float windSpeed = 12.f,
        float depth = 10000.f,
        float reflectionDamping = 0.35f,
        float scale = 1e-8,
        bool  isChoppy = true,
        float choppyFactor = -2.5f,
        float crestFoamHeight = 2.2f,
        bool  useVBO=false );

    void build( 
        const osg::Vec2f& windDirection,
        float windSpeed,
        float depth,
        float reflectionDamping,
        float waveScale,
        bool  isChoppy,
        float choppyFactor,
        float crestFoamHeight,
        bool  useVBO );

    void changeScene( SCENE_TYPE type );

    // Load the islands model
    // Here we attach a custom shader to the model.
    // This shader overrides the default shader applied by OceanScene but uses uniforms applied by OceanScene.
    // The custom shader is needed to add multi-texturing and bump mapping to the terrain.
    osg::Node* loadIslands(void);

    osg::ref_ptr<osg::TextureCubeMap> loadCubeMapTextures( const std::string& dir );

    osg::Geode* sunDebug( const osg::Vec3f& position );

    inline osg::Vec4f intColor(unsigned r, unsigned g, unsigned b, unsigned a = 255 )
    {
        float div = 1.f/255.f;
        return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
    }

    inline osgOcean::OceanScene::EventHandler* getOceanSceneEventHandler()
    {
        return _oceanScene->getEventHandler();
    }

    inline osgOcean::OceanTechnique* getOceanSurface( void )
    {
        return _oceanSurface.get();
    }

    inline osg::Group* getScene(void){
        return _scene.get();
    }

    inline osgOcean::OceanScene* getOceanScene()
    {
        return _oceanScene.get();
    }
};