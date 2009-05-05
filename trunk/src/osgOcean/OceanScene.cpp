#include <osgOcean/OceanScene>

using namespace osgOcean;

OceanScene::OceanScene( void ):
	_oceanSurface			( 0 ),
	_isDirty					( true ),
	_enableReflections	( false ),
	_enableRefractions	( false ),
	_reflectionTexSize	( 512,512 ),
	_refractionTexSize	( 512,512 ),
	_reflectionUnit		( 1 ),
	_refractionUnit		( 2 ),
	_reflectionSceneMask	( 0x1 ),
	_refractionSceneMask	( 0x2 ),
	_normalSceneMask		( 0x4 ),
	_surfaceMask			( 0x8 ),
	_lightID					( 0 ),
	_underwaterFogColor	( 0.2274509f, 0.4352941f, 0.7294117f, 1.f ),
	_surfaceStateSet		( new osg::StateSet ),
	_reflectionMatrix		( 1,  0,  0,  0,
								  0,  1,  0,  0,
								  0,  0, -1,  0,	
								  0,  0,  0,  1 )
{
	setNumChildrenRequiringUpdateTraversal(1);
}

OceanScene::OceanScene( OceanTechnique* technique ):
	_oceanSurface			( technique ),
	_isDirty					( true ),
	_enableReflections	( false ),
	_enableRefractions	( false ),
	_reflectionTexSize	( 512,512 ),
	_refractionTexSize	( 512,512 ),
	_reflectionUnit		( 1 ),
	_refractionUnit		( 2 ),
	_reflectionSceneMask	( 0x1 ),
	_refractionSceneMask	( 0x2 ),
	_normalSceneMask		( 0x4 ),
	_surfaceMask			( 0x8 ),
	_lightID					( 0 ),
	_underwaterFogColor	( 0.2274509f, 0.4352941f, 0.7294117f, 1.f ),
	_surfaceStateSet		( new osg::StateSet ),
	_reflectionMatrix		( 1,  0,  0,  0,
								  0,  1,  0,  0,
								  0,  0, -1,  0,	
								  0,  0,  0,  1 )
{
	_oceanSurface->setNodeMask( _surfaceMask );
	addChild( _oceanSurface.get() );

	setNumChildrenRequiringUpdateTraversal(1);
}

OceanScene::OceanScene( const OceanScene& copy, const osg::CopyOp& copyop ):
	osg::Group				( copy,copyop ),
	_oceanSurface			( copy._oceanSurface ),
	_isDirty					( copy._isDirty ),
	_enableReflections	( copy._enableReflections ),
	_enableRefractions	( copy._enableRefractions ),
	_reflectionTexSize	( copy._reflectionTexSize ),
	_refractionTexSize	( copy._refractionTexSize ),
	_reflectionUnit		( copy._reflectionUnit ),
	_refractionUnit		( copy._refractionUnit ),
	_reflectionSceneMask	( copy._reflectionSceneMask ),
	_refractionSceneMask	( copy._refractionSceneMask ),
	_surfaceMask			( copy._surfaceMask ),
	_normalSceneMask		( copy._normalSceneMask),
	_lightID					( copy._lightID ),
	_reflectionMatrix		( copy._reflectionMatrix),
	_surfaceStateSet		( copy._surfaceStateSet ),
	_underwaterFogColor	( copy._underwaterFogColor )
{
}

OceanScene::~OceanScene( void )
{

}

void OceanScene::init( void )
{
	_refractionCamera = NULL;
	_reflectionCamera = NULL;

	_surfaceStateSet = new osg::StateSet;

	if( _oceanSurface.valid() )
	{
		_surfaceStateSet->addUniform( new osg::Uniform("uLightID", _lightID ) );
		_surfaceStateSet->addUniform( new osg::Uniform("uEnableDOF", false ) );
		_surfaceStateSet->addUniform( new osg::Uniform("uEnableReflections", _enableReflections ) );
		_surfaceStateSet->addUniform( new osg::Uniform("uEnableRefractions", _enableRefractions ) );
		
		_surfaceStateSet->addUniform( new osg::Uniform("uReflectionMap", _reflectionUnit ) );	
		_surfaceStateSet->addUniform( new osg::Uniform("uRefractionMap", _refractionUnit ) );
		_surfaceStateSet->addUniform( new osg::Uniform("uUnderwaterFogColor", _underwaterFogColor ) );

		if( _enableReflections )
		{
			osg::Texture2D* reflectionTexture = createTextureRGBA( _reflectionTexSize );
			_reflectionCamera = reflectionPass( reflectionTexture );
			_surfaceStateSet->setTextureAttributeAndModes( _reflectionUnit, reflectionTexture, osg::StateAttribute::ON );
		}

		if( _enableRefractions )
		{
			osg::Texture2D* refractionTexture = createTextureRGBA( _refractionTexSize );
			_refractionCamera = refractionPass( refractionTexture );
			_surfaceStateSet->setTextureAttributeAndModes( _refractionUnit, refractionTexture, osg::StateAttribute::ON );
		}
	}
	
	_isDirty = false;
}

osg::Camera* OceanScene::reflectionPass( osg::Texture2D* texture )
{
	osg::Camera* camera = new osg::Camera;

	camera->setClearColor( osg::Vec4(0.0, 0.0, 0.0, 0.0) );
	camera->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
	camera->setViewport(0, 0, _reflectionTexSize.x(), _reflectionTexSize.y());

	camera->setRenderOrder(osg::Camera::PRE_RENDER);
	camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	camera->attach(osg::Camera::COLOR_BUFFER,  texture);

	camera->setCullCallback( new CameraCullCallback(this) );
	camera->setCullMask(_reflectionSceneMask);

	return camera;
}

osg::Camera* OceanScene::refractionPass( osg::Texture2D* texture )
{
	osg::Camera* camera = new osg::Camera;

	camera->setClearColor( osg::Vec4( 0.160784, 0.231372, 0.325490, 0.0 ) );
	camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );

	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF_INHERIT_VIEWPOINT );
	camera->setViewport(0, 0, _refractionTexSize.x(), _refractionTexSize.y() );

	camera->setRenderOrder(osg::Camera::PRE_RENDER);
	camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	camera->attach( osg::Camera::COLOR_BUFFER,  texture );

	camera->setCullCallback( new CameraCullCallback(this) );
	camera->setCullMask( _refractionSceneMask );

	return camera;
}

osg::Camera* OceanScene::surfacePass( osg::StateSet* ss )
{
	osg::Camera* camera = new osg::Camera;
	camera->setReferenceFrame( osg::Transform::RELATIVE_RF );
	camera->setRenderOrder( osg::Camera::NESTED_RENDER );
	camera->setCullCallback( new CameraCullCallback(this) );
	camera->setCullMask( _surfaceMask );
	camera->setStateSet( ss );

	return camera;
}

osg::Texture2D* OceanScene::createTextureRGBA( const osg::Vec2s& size )
{
	osg::Texture2D* texture2D = new osg::Texture2D();

	texture2D->setTextureSize(size.x(), size.y());
	texture2D->setInternalFormat(GL_RGBA);
	texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
	texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	texture2D->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP );
	texture2D->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP );
	texture2D->setDataVariance(osg::Object::DYNAMIC);

	return texture2D;
}

void OceanScene::traverse( osg::NodeVisitor& nv )
{
	if( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
	{
		if( _isDirty )
			init();

		osg::Group::traverse(nv);
	}
	else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
		
		if (cv) 
			cull(*cv);
		else
			osg::Group::traverse(nv);
	}
	else
		osg::Group::traverse(nv);
}

void OceanScene::cull(osgUtil::CullVisitor& cv)
{
	if(_oceanSurface.valid() )
	{
		if( cv.getEyePoint().z() > _oceanSurface->getSurfaceHeight() )
		{
			if( _reflectionCamera.valid() )	
			{
				// update reflection camera and render reflected scene
				_reflectionCamera->setViewMatrix( _reflectionMatrix * cv.getCurrentCamera()->getViewMatrix() );
				_reflectionCamera->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );
				_reflectionCamera->accept( cv );
			}
		}
		else
		{
			if( _refractionCamera.valid() )
			{
				// update refraction camera and render refracted scene
				_refractionCamera->setViewMatrix( cv.getCurrentCamera()->getViewMatrix() );
				_refractionCamera->setProjectionMatrix( cv.getCurrentCamera()->getProjectionMatrix() );
				_refractionCamera->accept( cv );
			}
		}

		unsigned int mask = cv.getTraversalMask();

		// render ocean surface with reflection / refraction stateset
		cv.pushStateSet( _surfaceStateSet.get() );
		cv.setTraversalMask( mask & _surfaceMask );
		osg::Group::traverse(cv);
		cv.popStateSet();

		// render rest of scene
		cv.setTraversalMask( mask & _normalSceneMask );
		osg::Group::traverse(cv);
		
		// put original mask back
		cv.setTraversalMask( mask );
	}
	else
		osg::Group::traverse(cv);
}

// -------------------------------
//     Callback implementation
// -------------------------------

OceanScene::CameraCullCallback::CameraCullCallback(OceanScene* st):
	_oceanScene(st)
{
}

void OceanScene::CameraCullCallback::operator()(osg::Node*, osg::NodeVisitor* nv)
{
	_oceanScene->osg::Group::traverse(*nv);
}