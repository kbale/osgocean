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

#include <osgOcean/GodRayBlendSurface>
#include <osgOcean/ShaderManager>

using namespace osgOcean;

#define USE_LOCAL_SHADERS 1

GodRayBlendSurface::GodRayBlendSurface( void ):
    _HGg          (1.f-(.2f*.2f), 1.f+(.2f*.2f), 2.f*.2f),
    _intensity    (0.2f),
    _sunDir       (0.f,0.f,-1.f)
{
    
}

GodRayBlendSurface::GodRayBlendSurface( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture ):
    _HGg        (1.f-(.2f*.2f), 1.f+(.2f*.2f), 2.f*.2f),
    _intensity  (0.2f),
    _sunDir     (0.f,0.f,-1.f)
{
    build(corner,dims,texture);
}

GodRayBlendSurface::GodRayBlendSurface( const GodRayBlendSurface &copy, const osg::CopyOp &copyop ):
    osg::Geode        (copy,copyop),
    _HGg              (copy._HGg),
    _intensity        (copy._intensity),
    _sunDir           (copy._sunDir),
    _stateset         (copy._stateset),
    _normalArray      (copy._normalArray )
{

}

void GodRayBlendSurface::build( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture )
{
    removeDrawables( 0, getNumDrawables() );

    osg::Geometry* geom = new ScreenAlignedQuad(corner,dims,texture);

    geom->setUseDisplayList(false);
    geom->setDataVariance(osg::Object::DYNAMIC);

    _normalArray = new osg::Vec3Array(4);
    geom->setNormalArray(_normalArray.get());
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    addDrawable(geom);

    _stateset = new osg::StateSet;

    osg::ref_ptr<osg::Program> program = createShader();

    if(program.valid())
        _stateset->setAttributeAndModes( program.get(), osg::StateAttribute::ON );

    osg::BlendFunc *blend = new osg::BlendFunc;
    blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE);

    _stateset->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON);
    _stateset->setAttributeAndModes(blend, osg::StateAttribute::ON);
    _stateset->setMode(GL_BLEND, osg::StateAttribute::ON );

    _stateset->addUniform( new osg::Uniform("osgOcean_GodRayTexture", 0 ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_Eye",           osg::Vec3f(0.f, 0.f,  0.f ) ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_ViewerDir",     osg::Vec3f(0.f, 1.f,  0.f ) ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_SunDir",        osg::Vec3f(0.f, 0.f, -1.f ) ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_HGg",           _HGg ) );
    _stateset->addUniform( new osg::Uniform("osgOcean_Intensity",     _intensity ) );

    setStateSet( _stateset.get() );

    setUserData( new GodRayBlendDataType(*this) );
    setCullCallback( new GodRayBlendCallback );
    setUpdateCallback( new GodRayBlendCallback );
}

osg::Program* GodRayBlendSurface::createShader(void)
{
#if USE_LOCAL_SHADERS

    static const char god_ray_screen_blend_vertex[] = 
        "varying vec3 vRay;\n"
        "uniform vec3 osgOcean_Eye;\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    gl_Position = gl_Vertex;\n"
        "\n"
        "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
        "\n"
        "    gl_TexCoord[1].xy = gl_TexCoord[0].st;\n"
        "    gl_TexCoord[1].zw = gl_TexCoord[0].st + vec2(1.0, 0.0);\n"
        "    gl_TexCoord[2].xy = gl_TexCoord[0].st + vec2(1.0, 1.0);\n"
        "    gl_TexCoord[2].zw = gl_TexCoord[0].st + vec2(0.0, 1.0);\n"
        "\n"
        "    vRay = gl_Normal;\n"
        "}\n";

    static const char god_ray_screen_blend_fragment[] =
        "uniform sampler2DRect osgOcean_GodRayTexture;\n"
        "\n"
        "uniform vec3    osgOcean_SunDir;\n"
        "uniform vec3    osgOcean_HGg;\n"
        "uniform float    osgOcean_Intensity;\n"
        "uniform vec3  osgOcean_Eye;\n"
        "\n"
        "varying vec3 vRay;\n"
        "\n"
        "const float bias = 0.15; // used to hide aliasing\n"
        "\n"
        "float computeMie(vec3 viewDir, vec3 sunDir) \n"
        "{\n"
        "    float num = osgOcean_HGg.x;\n"
        "    float den = (osgOcean_HGg.y - osgOcean_HGg.z*dot(sunDir, viewDir)); \n"
        "    den = inversesqrt(den); \n"
        "\n"
        "    float phase = num * (den*den*den);\n"
        "\n"
        "    return phase;\n"
        "}\n"
        "\n"
        "void main( void )\n"
        "{\n"
        "    vec4 shafts;\n"
        "\n"
        "    shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].xy);\n"
        "    shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[1].zw);\n"
        "    shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].xy);\n"
        "    shafts += texture2DRect(osgOcean_GodRayTexture, gl_TexCoord[2].zw);\n"
        "\n"
        "    shafts /= 4.0;\n"
        "\n"
        "    vec3 rayNormalised = normalize(vRay-osgOcean_Eye);\n"
        "\n"
        "    float phase = computeMie(rayNormalised, -osgOcean_SunDir);\n"
        "\n"
        "    // Calculate final color, adding a little bias (0.15 here)\n"
        "    // to hide aliasing\n"
        "    vec3 colour = (bias+osgOcean_Intensity*shafts.rgb)*phase;\n"
        "\n"
        "    vec3 ray = ( rayNormalised + vec3(1.0) ) / 2.0; \n"
        "\n"
        "    gl_FragColor = vec4(colour, 1.0);\n"
        "}\n";

#else
    static const char god_ray_screen_blend_vertex[]   = "godray_screen_blend.vert";
    static const char god_ray_screen_blend_fragment[] = "godray_screen_blend.frag";
#endif

    return ShaderManager::instance().createProgram("godray_blend", god_ray_screen_blend_vertex, god_ray_screen_blend_fragment, !USE_LOCAL_SHADERS );
}

void GodRayBlendSurface::update( const osg::Matrixd& view, const osg::Matrixd& proj )
{
    // Get the corners of the far clipping plane.
    double far     = proj(3,2) / (1.0+proj(2,2));

    double fLeft   = far * (proj(2,0)-1.0) / proj(0,0);
    double fRight  = far * (1.0+proj(2,0)) / proj(0,0);
    double fTop    = far * (1.0+proj(2,1)) / proj(1,1);
    double fBottom = far * (proj(2,1)-1.0) / proj(1,1);

    // multiply plane coords by inverse view matrix and add 
    // to the normal array. These will be used as the ray 
    // vectors for the view direction at the corners of the 
    // screen aligned quad
    osg::Matrixd inv_view = osg::Matrixd::inverse(view);

    (*_normalArray)[0] = osg::Vec3d( fLeft,  fTop,    -far ) * inv_view;
    (*_normalArray)[1] = osg::Vec3d( fLeft,  fBottom, -far ) * inv_view;
    (*_normalArray)[2] = osg::Vec3d( fRight, fBottom, -far ) * inv_view;
    (*_normalArray)[3] = osg::Vec3d( fRight, fTop,    -far ) * inv_view;
}

// --------------------------------------------
//          Callback implementations
// --------------------------------------------

GodRayBlendSurface::GodRayBlendDataType::GodRayBlendDataType(GodRayBlendSurface& blendSurface):
    _blendSurface( blendSurface )
{}

GodRayBlendSurface::GodRayBlendDataType::GodRayBlendDataType(const GodRayBlendDataType& copy, 
                                                                                 const osg::CopyOp& copyop ):
    _blendSurface (copy._blendSurface),
    _view         (copy._view),
    _projection   (copy._projection)
{}

void GodRayBlendSurface::GodRayBlendDataType::update(void)
{
    _blendSurface.update(_view,_projection);
}

void GodRayBlendSurface::GodRayBlendCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::ref_ptr<GodRayBlendDataType> data = dynamic_cast<GodRayBlendDataType*> ( node->getUserData() );

    if(data.valid())
    {
        if( nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);

            // get view and projection matrices for the main view camera
            data->setViewMatrix(cv->getRenderStage()->getCamera()->getViewMatrix());
            data->setProjMatrix(cv->getRenderStage()->getCamera()->getProjectionMatrix());
        }
        else if(nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
        {
            data->update();
        }
    }

    traverse(node, nv); 
}