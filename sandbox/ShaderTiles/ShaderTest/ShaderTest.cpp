// ShaderTest.cpp : Defines the entry point for the console application.
//

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Geometry>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgGA/GUIEventHandler>



osg::Geometry* createGeometry( osg::Vec3f pos )
{
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back( osg::Vec3f(-10.f, -10.f, 0.f ) );
    vertices->push_back( osg::Vec3f( 10.f, -10.f, 0.f ) );
    vertices->push_back( osg::Vec3f( 10.f,  10.f, 0.f ) );
    vertices->push_back( osg::Vec3f(-10.f,  10.f, 0.f ) );

    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back( osg::Vec3f( 0.f, 0.f, 1.f ) );

    osg::Vec4Array* color = new osg::Vec4Array;
    color->push_back( osg::Vec4f( pos.x(), pos.y(), pos.z(), 1.f ) );

    osg::DrawElementsUInt* prim = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    prim->push_back(0);
    prim->push_back(1);
    prim->push_back(2);
    prim->push_back(3);

    osg::BoundingBox bb;

    bb.xMin() = pos.x() - 10.f;
    bb.xMax() = pos.x() + 10.f;
    bb.yMin() = pos.y() - 10.f;
    bb.yMin() = pos.y() + 10.f;
    bb.zMin() = pos.z() - 1.f;
    bb.zMax() = pos.z() + 1.f;

    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray(vertices);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geom->setColorArray(color);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(prim);
    geom->setInitialBound( bb );

    return geom;
}

osg::Program* createShader( void )
{
    static const char vertexShader[] = {
        "void main(void)\n"
        "{\n"
        "vec4 v = gl_Vertex;\n"
        "v.xyz += gl_Color.xyz;\n"
        "gl_Position = gl_ModelViewProjectionMatrix * v;\n"
        "};"
    };

    static const char fragmentShader[] = {
        "void main(void){\n"
        "gl_FragColor = vec4(1.0);\n"
        "}"
    };

    osg::Shader* vshader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fshader = new osg::Shader(osg::Shader::FRAGMENT);

    vshader->setShaderSource( vertexShader );
    fshader->setShaderSource( fragmentShader );

    osg::Program* program = new osg::Program;
    program->addShader( vshader );
    program->addShader( fshader );

    return program;
}

int main(int argc, char* argv[])
{
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(25,25,900,700,0);
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet() ) );
    viewer.getCamera()->setClearColor( osg::Vec4f( (1.f/255.f)*153.f, (1.f/255.f)*217.f, (1.f/255.f)*234.f,1) );
    
    osg::Geode* geode = new osg::Geode;
    
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes( createShader(), osg::StateAttribute::ON );

    for(int r = 0; r < 50; ++r)
    {
        for(int c = 0; c < 50; ++c )
        {
            geode->addDrawable( createGeometry( osg::Vec3f( r*10.f, -c*10.f, 0.f ) ) );
        }
    }

    osg::Group* root = new osg::Group;

    root->addChild( geode );

    viewer.setSceneData(root);

    viewer.run();

	return 0;
}

