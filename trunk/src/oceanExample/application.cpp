
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/FlightManipulator>
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
#include <osg/Fog>

#include <string>
#include <vector>

#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>

#include "SkyDome.h"

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
		title->setLineSpacing(-0.35f);
		title->setText("osgOcean\nPress 1-3 to change presets\nPress 'C' to change camera");
		textGeode->addDrawable( title );

		_modeText = new osgText::Text;
		_modeText->setPosition( osg::Vec3f(0.f, -60.f, 0.f ) );
		_modeText->setDataVariance(osg::Object::DYNAMIC);
		textGeode->addDrawable( _modeText );

		_cameraModeText = new osgText::Text;
		_cameraModeText->setPosition( osg::Vec3f(0.f, -80.f, 0.f ) );
		_cameraModeText->setDataVariance(osg::Object::DYNAMIC);
		textGeode->addDrawable( _cameraModeText );

		osg::PositionAttitudeTransform* titlePAT = new osg::PositionAttitudeTransform;
		titlePAT->setPosition( osg::Vec3f( 10, 90, 0.f ) );
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

	osg::Camera* getHudCamera(void){
		return _camera.get();
	}
};

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
	osg::ref_ptr<osg::Fog> _fog;
	osg::ref_ptr<osg::TextureCubeMap> _cubemap;
	osg::ref_ptr<SkyDome> _skyDome;
	osg::ref_ptr<osg::Vec4Array> _oceanBoxColor;
		
	std::vector<std::string> _cubemapDirs;
	std::vector<osg::Vec4f>  _lightColors;
	std::vector<osg::Vec4f>  _fogColors;
		
	osg::ref_ptr<osg::Light> _light;

	std::vector<osg::Vec3f>  _sunPositions;
	std::vector<osg::Vec4f>  _sunDiffuse;
	std::vector<osg::Vec4f>  _waterfogColors;


public:
	SceneModel( void ):
		_sceneType(CLEAR)
	{
		_cubemapDirs.push_back( "sky_clear" );
		_cubemapDirs.push_back( "sky_dusk" );
		_cubemapDirs.push_back( "sky_fair_cloudy" );

		_fogColors.push_back( intColor( 195,221,254 ) );
		_fogColors.push_back( intColor( 248,241,189 ) );
		_fogColors.push_back( intColor( 172,224,251 ) );

		_lightColors.push_back( intColor( 105,138,174 ) );
		_lightColors.push_back( intColor( 105,138,174 ) );
		_lightColors.push_back( intColor( 105,138,174 ) );

		_sunPositions.push_back( osg::Vec3f(410.f, 1480.f, 1510.f) );
		_sunPositions.push_back( osg::Vec3f(520.f, 1900.f, 550.f ) );
		_sunPositions.push_back( osg::Vec3f(410.f, 1480.f, 1510.f) );

		_sunDiffuse.push_back( intColor( 191, 191, 191 ) );
		_sunDiffuse.push_back( intColor( 251, 251, 161 ) );
		_sunDiffuse.push_back( intColor( 251, 251, 161 ) );

		_waterfogColors.push_back( intColor(70,113,174 ) );
		_waterfogColors.push_back( intColor(44,69,106 ) );
		_waterfogColors.push_back( intColor(84,135,172 ) );

		build();
	}

	void build( void )
	{
		_scene = new osg::Group; 

		_cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );

		// Set up fogging
		_fog = new osg::Fog;
		_fog->setStart(0.f);
		_fog->setEnd(1800.f);
		_fog->setMode(osg::Fog::LINEAR);
		_fog->setColor( _fogColors[_sceneType] ); 
		
		// Set up surface
		_oceanSurface = new osgOcean::FFTOceanSurface( 64, 256, 15, osg::Vec2f(1.1f,1.1f), 12.f, 10000.f, true, -2.5f, 10.f, 256 );		
		_oceanSurface->setEnvironmentMap( _cubemap.get() );
		_oceanSurface->setFoamHeight( 2.2f );
		_oceanSurface->enableCrestFoam( true );
		_oceanSurface->setLightColor( _lightColors[_sceneType] );

		// Set up ocean scene, add surface
		_oceanScene = new osgOcean::OceanScene( _oceanSurface.get() );
		_oceanScene->setLightID(0);
		_oceanScene->enableReflections(true);
		_oceanScene->enableRefractions(true);
		_oceanScene->getOrCreateStateSet()->setAttributeAndModes(_fog.get(),osg::StateAttribute::ON);
		_oceanScene->setUnderwaterFogColor( _waterfogColors[_sceneType] );

		// create sky dome and add to ocean scene
		_skyDome = new SkyDome( 2000.f, 16, 16, _cubemap.get() );
		// set masks so it appears in reflected scene and normal scene
		_skyDome->setNodeMask( _oceanScene->getReflectedSceneMask() | _oceanScene->getNormalSceneMask() );

		osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform;
		pat->setPosition( osg::Vec3f(0.f, 0.f, -1.f ) );
		pat->addChild( _skyDome.get() );
				
		_oceanScene->addChild( pat );

		_scene->addChild( _oceanScene.get() );

		osg::LightSource* lightSource = new osg::LightSource;
		lightSource->setLocalStateSetModes();

		_light = lightSource->getLight();
		_light->setLightNum(0);
		_light->setAmbient( osg::Vec4d(0.3f, 0.3f,	0.3f,	1.0f ));
		_light->setDiffuse( _sunDiffuse[_sceneType] );
		_light->setSpecular(osg::Vec4d(1.0f, 1.0f,	1.0f,	1.0f ));
		_light->setPosition( osg::Vec4f(_sunPositions[_sceneType],1.f) ); // point light

		_scene->addChild( lightSource );

		_oceanBoxColor = new osg::Vec4Array(1);
		_oceanBoxColor->at(0) = _waterfogColors[_sceneType];

		_scene->addChild( oceanBox( _oceanBoxColor.get() ) );
	}

	osg::Group* getScene(void){
		return _scene.get();
	}

	void changeScene( SCENE_TYPE type )
	{
		_sceneType = type;

		_fog->setColor( _fogColors[_sceneType] ); 

		_cubemap = loadCubeMapTextures( _cubemapDirs[_sceneType] );
		
		_skyDome->setCubeMap( _cubemap.get() );
		
		_oceanSurface->setEnvironmentMap( _cubemap.get() );
		_oceanSurface->setLightColor( _lightColors[type] );
		_oceanScene->setUnderwaterFogColor( _waterfogColors[_sceneType] );

		_light->setPosition( osg::Vec4f(_sunPositions[_sceneType],1.f) );
		_light->setDiffuse( _sunDiffuse[_sceneType] ) ;

		_oceanBoxColor->at(0) = _waterfogColors[_sceneType];
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

		cubeMap->setFilter( osg::Texture::MIN_FILTER,	osg::Texture::LINEAR_MIPMAP_LINEAR);
		cubeMap->setFilter( osg::Texture::MAG_FILTER,	osg::Texture::LINEAR);
		cubeMap->setWrap	( osg::Texture::WRAP_S,			osg::Texture::CLAMP_TO_EDGE);
		cubeMap->setWrap	( osg::Texture::WRAP_T,			osg::Texture::CLAMP_TO_EDGE);

		cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_X, osgDB::readImageFile( filenames[NEG_X] ) );
		cubeMap->setImage(osg::TextureCubeMap::POSITIVE_X, osgDB::readImageFile( filenames[POS_X] ) );
		cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Y, osgDB::readImageFile( filenames[NEG_Y] ) );
		cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Y, osgDB::readImageFile( filenames[POS_Y] ) );
		cubeMap->setImage(osg::TextureCubeMap::NEGATIVE_Z, osgDB::readImageFile( filenames[NEG_Z] ) );
		cubeMap->setImage(osg::TextureCubeMap::POSITIVE_Z, osgDB::readImageFile( filenames[POS_Z] ) );

		return cubeMap;
	}

	osg::Geode* oceanBox( osg::Vec4Array* color )
	{
		osg::Geode* geode = new osg::Geode;
		
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		osg::Geometry* geom = new osg::Geometry;
		geom->setUseDisplayList(false);
		
		osg::Vec3Array* vertices = new osg::Vec3Array;

		// bottom set;
		vertices->push_back( osg::Vec3f(-2000.f, -2000.f, -1000.f ) );	// A
		vertices->push_back( osg::Vec3f( 2000.f, -2000.f, -1000.f ) ); // B
		vertices->push_back( osg::Vec3f( 2000.f,  2000.f, -1000.f ) ); // C
		vertices->push_back( osg::Vec3f(-2000.f,  2000.f, -1000.f ) ); // D
		// top set
		vertices->push_back( osg::Vec3f(-2000.f, -2000.f, 10.f ) ); // E
		vertices->push_back( osg::Vec3f( 2000.f, -2000.f, 10.f ) ); // F
		vertices->push_back( osg::Vec3f( 2000.f,  2000.f, 10.f ) ); // G
		vertices->push_back( osg::Vec3f(-2000.f,  2000.f, 10.f ) ); // H

		osg::DrawElementsUInt* prim = new osg::DrawElementsUInt(osg::PrimitiveSet::QUAD_STRIP,0);
		prim->push_back( 0 );
		prim->push_back( 4 );
		prim->push_back( 1 );
		prim->push_back( 5 );
		prim->push_back( 2 );
		prim->push_back( 6 );
		prim->push_back( 3 );
		prim->push_back( 7 );
		prim->push_back( 0 );
		prim->push_back( 4 );

		geom->addPrimitiveSet( prim );

		osg::DrawElementsUInt* base =  new osg::DrawElementsUInt( osg::PrimitiveSet::QUADS, 0 );
		base->push_back( 0 );
		base->push_back( 3 );
		base->push_back( 2 );
		base->push_back( 1 );

		geom->addPrimitiveSet( base );

		geom->setVertexArray( vertices );
		geom->setColorArray( color );
		geom->setColorBinding( osg::Geometry::BIND_OVERALL );

		geode->addDrawable( geom );
		
		return geode;
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
};


class SceneEventHandler : public osgGA::GUIEventHandler
{
private:
	osg::ref_ptr<SceneModel> _scene;
	osg::ref_ptr<TextHUD> _textHUD;
	osgViewer::Viewer& _viewer;
	bool _isFixedCamera;

public:
	SceneEventHandler( SceneModel* scene, TextHUD* textHUD, osgViewer::Viewer& viewer ):
		_scene(scene),
		_textHUD(textHUD),
		_viewer(viewer),
		_isFixedCamera(true)
	{
		_textHUD->setSceneText("Clear");
		_textHUD->setCameraText("Fixed");
		_viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
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
					_isFixedCamera = !_isFixedCamera;

					if(_isFixedCamera){
						_viewer.getCamera()->setViewMatrixAsLookAt( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
						_viewer.setCameraManipulator(NULL);
						_textHUD->setCameraText("Fixed");
					}
					else{
						osgGA::FlightManipulator* flight = new osgGA::FlightManipulator;
						flight->setHomePosition( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
						_viewer.setCameraManipulator( flight );
						_textHUD->setCameraText("Flight");
					}
				}
			}
		default:
			return false;
		}
	}
};

int main(int argc, char *argv[])
{
	osgViewer::Viewer viewer;
	osgGA::FlightManipulator* flight = new osgGA::FlightManipulator;
	flight->setHomePosition( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
	viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
	viewer.setUpViewInWindow( 25,25,1024,768, 0 );
	viewer.addEventHandler(new osgViewer::StatsHandler );
	
	osg::ref_ptr<TextHUD> hud = new TextHUD;
	osg::ref_ptr<SceneModel> scene = new SceneModel;
	viewer.addEventHandler( new SceneEventHandler(scene.get(), hud.get(), viewer ) );

	osg::Group* root = new osg::Group;
	root->addChild( scene->getScene() );
	root->addChild( hud->getHudCamera() );

	viewer.setSceneData( root );

	viewer.realize();

	while(!viewer.done())
	{
		viewer.frame();	
	}
}
