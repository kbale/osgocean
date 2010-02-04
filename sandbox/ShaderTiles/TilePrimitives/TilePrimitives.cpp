
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>

#include <vector>
#include "MipmapGeometry.h"

#define NUM_TILES 15
#define PRINT osg::notify(osg::NOTICE)
#define ENDPRINT std::endl

osg::ref_ptr<osg::Vec3Array> _myActiveVertices;

static const char vertexShader[] =
{
    "void main(void)\n"
    "{\n"
    "vec4 v = gl_Vertex;\n"
    "v.xyz += gl_Color.xyz;\n"
    "gl_Position = gl_ModelViewProjectionMatrix * v;\n"
    "};"
};

static const char fragmentShader[] = 
{
    "void main(void){\n"
    "gl_FragColor = gl_Color;\n"
    "}"
};

osg::Vec3Array* createVertices( unsigned size, float spacing, float repeats )
{   
    osg::Vec3Array* vertices = new osg::Vec3Array;

    float div = repeats / (float)(size+1);
    
    for(unsigned r = 0; r < size+1; ++r )
    {
        for(unsigned c = 0; c < size+1; ++c)
        {
            osg::Vec3f v;

            v.x() = c * spacing;
            v.y() = r * -spacing;
            v.z() = sin( float(c)*div ) * 5.f;

            vertices->push_back(v);
        }
    }

    return vertices;
}

osg::Vec3Array* createNormals( unsigned size )
{   
    osg::Vec3Array* normals = new osg::Vec3Array;

    for(unsigned r = 0; r < size+1; ++r )
    {
        for(unsigned c = 0; c < size+1; ++c)
        {
            normals->push_back( osg::Vec3f(0,0,1) );
        }
    }

    return normals;
}

osg::PositionAttitudeTransform* createPAT( const osg::Vec3f& pos )
{
    osg::PositionAttitudeTransform* PAT = new osg::PositionAttitudeTransform;
    PAT->setPosition( pos );
    return PAT;
}

class DataType: public osg::Referenced
{
private:
    std::vector< osg::ref_ptr<osg::Vec3Array> >& _vertices;
    osg::ref_ptr<osg::Vec3Array> _activeVertices;
    osg::ref_ptr<osg::Geode> _geode;
public:
    DataType( osg::Geode* geode, std::vector< osg::ref_ptr<osg::Vec3Array> >& verts, osg::ref_ptr<osg::Vec3Array> activeVertices ):
      _activeVertices(activeVertices),
      _geode(geode),
      _vertices(verts)
    {}

    void update(unsigned frameNumber)
    {
        for(unsigned i = 0; i < _geode->getNumDrawables(); ++i )
        {
            osgOcean::MipmapGeometry* geom = static_cast<osgOcean::MipmapGeometry*>(_geode->getDrawable(i));
            //geom->updateVertices(_vertices[frameNumber%2]);
        }
    }
};

class UpdateCallback : public osg::NodeCallback
{
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    { 
        osg::ref_ptr<DataType> data = dynamic_cast<DataType*> ( node->getUserData() );
        if(data.valid())
        {
            data->update( nv->getTraversalNumber() );
        }
        traverse(node,nv);
    }
};

osg::Geode* createShaderScene( std::vector< std::vector< osg::ref_ptr< osgOcean::MipmapGeometry > > >& tiles,
                               std::vector< osg::ref_ptr<osg::Vec3Array> >& vertices, 
                               osg::Vec3Array* normals )
{
    osg::Geode* geode = new osg::Geode;
    osg::StateSet* ss =  geode->getOrCreateStateSet();

    osg::Shader* vshader = new osg::Shader(osg::Shader::VERTEX);
    osg::Shader* fshader = new osg::Shader(osg::Shader::FRAGMENT);

    vshader->setShaderSource( vertexShader );
    fshader->setShaderSource( fragmentShader );

    osg::Program* program = new osg::Program;
    program->addShader( vshader );
    program->addShader( fshader );

    ss->setAttributeAndModes( program, osg::StateAttribute::ON );

    //geode->setUserData( new DataType( geode, vertices, _myActiveVertices ) );
    //geode->setUpdateCallback( new UpdateCallback );

    for(int r = 0; r < NUM_TILES; ++r)
    {
        std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> > tileRow(NUM_TILES);
        
        for(int c = 0; c < NUM_TILES; ++c)
        {
            int centreX = -(NUM_TILES*256)/2;
            int centreY =  (NUM_TILES*256)/2;
            osg::Vec3f offset( centreX+c*256, centreY-r*256, 0.f ); 

            osgOcean::MipmapGeometry* tile = new osgOcean::MipmapGeometry( 7, 256.f );
            tile->setVertexArray( vertices.at(0) );
            tile->setNormalArray( normals );
            tile->setOffset( offset );

            osg::BoundingBoxf bb;

            bb.xMin() = (int)offset.x();
            bb.xMax() = (int)offset.x()+256;

            bb.yMin() = (int)offset.y()-256;
            bb.yMax() = (int)offset.y();

            bb.zMin() = (int)-15.f;
            bb.zMax() = (int)15.f;

            tile->setInitialBound(bb);
            
            tileRow.at(c)=tile;

            geode->addDrawable( tile );
        }

        tiles.push_back( tileRow );
    }

    return geode;
}


osg::Geometry* createCircle( float radius )
{
    float r = sqrt(radius);
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* prim = new osg::DrawElementsUInt( osg::DrawElementsUInt::LINE_LOOP, 0);
    const float DEG2RAD = 3.14159f/180.f;

    unsigned c = 0;

    for (int i=0; i < 360; i++)
    {
        float degInRad = i*DEG2RAD;

        osg::Vec3f v;
        v.x() = cos(degInRad)*r ;
        v.y() = sin(degInRad)*r ;
        v.z() = 0.f;

        vertices->push_back(v);
        prim->push_back(c); ++c;
    }

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back( osg::Vec4f(1.f, 0.f, 0.f, 1.f) );

    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back( osg::Vec3f(0.f,0.f,1.f) );

    geom->setVertexArray( vertices );
    geom->setNormalArray( normals );
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );
    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );
    geom->addPrimitiveSet(prim);

    return geom;
}

osg::Geode* createEyeMarkers( const std::vector<float> distances )
{
    osg::Geode* geode = new osg::Geode;

    for(unsigned i=0; i<distances.size();++i)    
    {
        geode->addDrawable(createCircle(distances.at(i)) );
    }

    return geode;
}

class UpdateClass 
{
private:
    osg::Vec3f _startPos;
    int _startX;
    int _startY;
    int _origX;
    int _origY;
    std::vector< std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> > >& _tiles;

public:
    UpdateClass( std::vector< std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> > >& tiles ):
      _startX(-(NUM_TILES*256)/2),
      _startY( (NUM_TILES*256)/2),
      _origX( _startX ),
      _origY( _startY ),
      _tiles( tiles )
    {};

    ~UpdateClass(){};

    void UpdateClass::updateLevels( osg::Vec3f& eye, osg::Geode* geode, 
                                    const std::vector<float>& distances )
    {
        int x_offset = 0;
        int y_offset = 0;

        bool _isEndless = true;
        unsigned _tileResolution = 256;
        int tileSize = _tileResolution;

        bool _updateOffsets = false;

        if(_isEndless)
        {
            int xMin = _startX;
            int yMin = _startY-(tileSize*NUM_TILES);

            x_offset = (int) ( (eye.x()-xMin) / (float)_tileResolution );
            y_offset = (int) ( (eye.y()-yMin) / (float)_tileResolution );

            x_offset -= (int)NUM_TILES/2;
            y_offset -= (int)NUM_TILES/2;

            if(x_offset != 0 || y_offset != 0)
            {
                PRINT << "Surface Move." << ENDPRINT;

                _startX += x_offset * tileSize; 
                _startY += y_offset * tileSize; 

                if(x_offset == -1)
                {
                    osg::Vec3f offset;

                    for(int r = 0; r < NUM_TILES; ++r)
                    {
                        std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> >& row = _tiles.at(r);

                        offset.x() = _startX;
                        offset.y() = _startY-r*256;
                        offset.z() = 0;

                        row.insert( row.begin(), row.back() );   // insert the 
                        row.pop_back(); 
                        row.front()->setOffset( offset );         // change offset
                    }
                }
                else if(x_offset == 1)
                {
                    osg::Vec3f offset;

                    for(int r = 0; r < NUM_TILES; ++r)
                    {
                        std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> >& row = _tiles.at(r);

                        offset.x() = _startX + ( (NUM_TILES-1)*256 );
                        offset.y() = _startY-r*tileSize;
                        offset.z() = 0;

                        row.insert( row.end(), row.front() );
                        row.erase( row.begin() );
                        row.back()->setOffset( offset );
                    }                   
                }
                
                if(y_offset == -1)
                {
                    _tiles.insert( _tiles.end(), _tiles.front() );
                    _tiles.erase( _tiles.begin() );
                    
                    osg::Vec3f offset;

                    for(int c = 0; c < NUM_TILES; c++ )
                    {
                        offset.x() = _startX + c * tileSize;
                        offset.y() = _startY-( (NUM_TILES-1)*tileSize );
                        offset.z() = 0;

                        _tiles.back().at(c)->setOffset(offset);
                    }
                }
                else if(y_offset == 1)
                {
                    _tiles.insert( _tiles.begin(), _tiles.back() );
                    _tiles.pop_back();

                    osg::Vec3f offset;

                    for(int c = 0; c < NUM_TILES; c++ )
                    {
                        offset.x() = _startX + c * tileSize;
                        offset.y() = _startY;
                        offset.z() = 0;

                        _tiles.front().at(c)->setOffset(offset);
                    }
                }
            }
        }

        unsigned updates=0;

        for(int r = NUM_TILES-1; r>=0; --r )
        {
            for(int c = NUM_TILES-1; c>=0; --c )
            {
                osgOcean::MipmapGeometry* curGeom = _tiles.at(r).at(c);
                osg::Vec3f centre = curGeom->getBound().center();

                float distanceToTile2 = (centre-eye).length2();

                unsigned mipmapLevel = 0;
                unsigned rightLevel  = 0;
                unsigned belowLevel  = 0;

                for( unsigned int m = 0; m < distances.size(); ++m )
                {
                    if( distanceToTile2 > distances.at(m) )
                        mipmapLevel = m;
                }

                if( c != NUM_TILES-1 && r != NUM_TILES-1 ){
                    osgOcean::MipmapGeometry* rightGeom = _tiles.at(r).at(c+1);
                    osgOcean::MipmapGeometry* belowGeom = _tiles.at(r+1).at(c);
                    rightLevel = rightGeom->getLevel();
                    belowLevel = belowGeom->getLevel();
                }
                else 
                {
                    if( c != NUM_TILES-1 ){
                        osgOcean::MipmapGeometry* rightGeom = _tiles.at(r).at(c+1);
                        rightLevel = rightGeom->getLevel();
                    }
                    else{
                        rightLevel = mipmapLevel;
                    }

                    if( r != NUM_TILES-1 ){
                        osgOcean::MipmapGeometry* belowGeom = _tiles.at(r+1).at(c);
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
        PRINT <<  "Updates: " << updates << "\n" << ENDPRINT;

        //for(unsigned r = 0; r < NUM_TILES; ++r)
        //{
        //    for(unsigned c = 0; c < NUM_TILES; ++c )
        //    {
        //        osgOcean::MipmapGeometry* geom =_tiles.at(r).at(c);
        //        
        //        if( geom->getResolution() == 2 && 
        //            geom->_levelRight != 6 && 
        //            geom->_levelBelow != 6 && 
        //            geom->_rightBorder.size() == 0 )
        //        {
        //            PRINT << " X |";
        //        }
        //        else
        //            PRINT << "   |"; 

        //        //PRINT << geom->_level << " " << geom->_levelRight  << " " << geom->_levelBelow << " | ";
        //    }

        //    PRINT << "\n-----------------------------------------------------------------------------------------------------------------" << ENDPRINT;
        //}
        //PRINT << ENDPRINT;
    }
};

class EventHandler : public osgGA::GUIEventHandler
{
public:
    EventHandler(osg::PositionAttitudeTransform* markers, 
                 UpdateClass* updateClass,
                 osg::Geode* tiles,
                 const std::vector<float> distances ):
        _pat(markers),
        _updateClass(updateClass),
        _tiles(tiles),
        _distances(distances)
    {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);
protected:
    osg::ref_ptr<osg::PositionAttitudeTransform> _pat;
    UpdateClass* _updateClass;
    std::vector<float> _distances;
    osg::ref_ptr<osg::Geode> _tiles;
};

bool EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    float speed = 6.f;

    if (ea.getHandled()) return false;

    switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Up)
            {
                osg::Vec3f pos = _pat->getPosition();
                pos.y()+=speed;
                _pat->setPosition(pos);

                osg::Vec3f eye = pos;
                //eye.z() = 0.f;
                _updateClass->updateLevels( eye, _tiles, _distances );
                return true;
            }
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)
            {
                osg::Vec3f pos = _pat->getPosition();
                pos.y()-=speed;
                _pat->setPosition(pos);

                osg::Vec3f eye = pos;
                //eye.z() = 0.f;
                _updateClass->updateLevels( eye, _tiles, _distances );
                return true;
            }
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
            {
                osg::Vec3f pos = _pat->getPosition();
                pos.x()-=speed;
                _pat->setPosition(pos);

                osg::Vec3f eye = pos;
                //eye.z() = 0.f;
                _updateClass->updateLevels( eye, _tiles, _distances );
                //PRINT << "L: " << eye.x() << " " << eye.y() << " " << eye.z() << ENDPRINT;
                return true;
            }
            if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
            {
                osg::Vec3f pos = _pat->getPosition();
                pos.x()+=speed;
                _pat->setPosition(pos);

                osg::Vec3f eye = pos;
                //eye.z() = 0.f;
                _updateClass->updateLevels( eye, _tiles, _distances );
                //PRINT << "R: " << eye.x() << " " << eye.y() << " " << eye.z() << ENDPRINT;
                return true;
            }
            if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Space )
            {
                osg::Vec3f eye = _pat->getPosition();
                eye.x() = 0.f;
                eye.y() = 0.f;

                _pat->setPosition(eye);
                _updateClass->updateLevels( eye, _tiles, _distances );
                return true;
            }
            break;
        }
    default:
        break;
    }

    return false;
}

int main(int argc, char* argv[])
{
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(25,25,900,700,0);
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.addEventHandler( new osgViewer::StatsHandler );
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet() ) );
    viewer.getCamera()->setClearColor( osg::Vec4f( (1.f/255.f)*153.f, (1.f/255.f)*217.f, (1.f/255.f)*234.f,1) );
    std::vector< osg::ref_ptr<osg::Vec3Array> > vertices;
    vertices.push_back( createVertices( 64, 4.f, 10.f ) );
    vertices.push_back( createVertices( 64, 4.f, 20.f ) );

    osg::ref_ptr<osg::Vec3Array> normals = createNormals( 64 );

    osg::Vec3f eye(0,0,10);
    
    osg::MatrixTransform* transform = new osg::MatrixTransform;
    osg::Matrixf rot = osg::Matrixf::rotate( osg::Quat( osg::inDegrees(90.f), osg::Vec3f(1.f,0.f,0.f)) );
    transform->setMatrix(rot);
    
    std::vector<float> distances;

    distances.push_back(0);
    distances.push_back(512);
    distances.push_back(768);
    distances.push_back(1024);
    distances.push_back(1280);
    distances.push_back(1536);
    distances.push_back(1792);

    std::vector< std::vector< osg::ref_ptr<osgOcean::MipmapGeometry> > > _tiles;
    UpdateClass updateClass(_tiles);

    for(unsigned l = 0; l < distances.size(); ++l){
          distances.at(l) *= distances.at(l);
    }

    osg::Geode* geode = createShaderScene( _tiles, vertices, normals );
    updateClass.updateLevels( eye, geode, distances );
        
    transform->addChild( geode );
    
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3f(),35.f);
    osg::ShapeDrawable* sphereDraw = new osg::ShapeDrawable;
    sphereDraw->setColor(osg::Vec4f(1.f,0.f,0.f,1.f));
    sphereDraw->setShape(sphere);

    osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform;
    pat->setDataVariance(osg::Object::DYNAMIC);
    pat->setPosition(osg::Vec3f(eye.x(), eye.y(), eye.z()));

    osg::Geode* sphereGeode = new osg::Geode;
    sphereGeode->addDrawable(sphereDraw);
    osg::Geode* markers = createEyeMarkers(distances);

    pat->addChild(sphereGeode);
    pat->addChild(markers);
    
    transform->addChild(pat);

    transform->setEventCallback( new EventHandler(pat,&updateClass,geode,distances) );

    viewer.setSceneData( transform );

    viewer.getCamera()->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    while(!viewer.done())
    {
        //PRINT << "x: " << geode->getBound().center().x() << " " << geode->getBound().center().y() << ENDPRINT;
        viewer.frame();
    }
	return 0;
}

