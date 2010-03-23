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

#include <osgOcean/Version>
#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/FFTOceanSurfaceVBO>
#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>

//#include "SkyDome.h"

#define USE_CUSTOM_SHADER

osg::Vec4f intColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255 )
{
   float div = 1.f/255.f;
   return osg::Vec4f( div*(float)r, div*(float)g, div*float(b), div*(float)a );
}

int main(int argc, char *argv[])
{
    osg::notify(osg::NOTICE) << "osgOcean " << osgOceanGetVersion() << std::endl << std::endl;

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
    arguments.getApplicationUsage()->addCommandLineOption("--disableShaders","Disable use of shaders for the whole application. Also disables most visual effects as they depend on shaders.");
    arguments.getApplicationUsage()->addCommandLineOption("--vbo","Enable use of VBO Ocean Surface Modelling.");
    arguments.getApplicationUsage()->addCommandLineOption("--isEndlessOcean","Enable use of endless ocean surface.");
    arguments.getApplicationUsage()->addCommandLineOption("--numTiles <numTiles>","Specify a tile size of each ocean tile. Default: 17.");
    arguments.getApplicationUsage()->addCommandLineOption("--tileSize <tileSize>","Specify a tile size of each ocean tile. Default: 64.");

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

    bool disableShaders = false;
    if (arguments.read("--disableShaders")) disableShaders = true;

    bool useVBO = false;
    if (arguments.read("--vbo")) useVBO = true;

    bool isEndlessOcean = false;
    if (arguments.read("--isEndlessOcean")) isEndlessOcean = true;

    int tileSize = 64;
    while (arguments.read("--tileSize", tileSize ) );

    int resolution = 256;
    while (arguments.read("--resolution", resolution));

    int numTiles = 17;
    while (arguments.read("--numTiles", numTiles));

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

    osgOcean::ShaderManager::instance().enableShaders(!disableShaders);

    osg::ref_ptr<osgOcean::OceanTechnique> oceanSurface; 
    oceanSurface->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    if (useVBO)
    {
        osg::notify(osg::NOTICE) << "Using VBO Ocean Technique" << std::endl;

        osgOcean::FFTOceanSurfaceVBO* oceanSurfaceVBO 
            = new osgOcean::FFTOceanSurfaceVBO( tileSize, resolution, numTiles, 
                                                windDirection, windSpeed, depth, 
                                                reflectionDamping, scale, 
                                                isChoppy, choppyFactor, 10.f, 256 );
        
        oceanSurfaceVBO->setFoamBottomHeight( 2.2f );
        oceanSurfaceVBO->setFoamTopHeight( 3.0f );
        oceanSurfaceVBO->enableCrestFoam( true );
        oceanSurfaceVBO->setLightColor( intColor( 105,138,174 ));
        // Make the ocean surface track with the main camera position, giving the illusion
        // of an endless ocean surface.
        oceanSurfaceVBO->enableEndlessOcean(isEndlessOcean);

        oceanSurface = oceanSurfaceVBO;
    }
    else
    {
        osg::notify(osg::NOTICE) << "NOT Using VBO Ocean Technique" << std::endl;

        osgOcean::FFTOceanSurface* oceanSurfaceFFT 
            = new osgOcean::FFTOceanSurface( tileSize, resolution, numTiles, 
                                             windDirection, windSpeed, depth, 
                                             reflectionDamping, scale, 
                                             isChoppy, choppyFactor, 10.f, 256 );

        oceanSurfaceFFT->setFoamBottomHeight( 2.2f );
        oceanSurfaceFFT->setFoamTopHeight( 3.0f );
        oceanSurfaceFFT->enableCrestFoam( true );
        oceanSurfaceFFT->setLightColor( intColor( 105,138,174 ));
        // Make the ocean surface track with the main camera position, giving the illusion
        // of an endless ocean surface.
        oceanSurfaceFFT->enableEndlessOcean(isEndlessOcean);

        oceanSurface = oceanSurfaceFFT;
    }
    
    osgOcean::ShaderManager::instance().setGlobalDefinition("osgOcean_LightID", 0); 

    osg::Vec3f eye(0.f,0.f,20.f);
    osg::Vec3f centre = eye+osg::Vec3f(0.f,1.f,0.f);
    osg::Vec3f up(0.f, 0.f, 1.f);
    
    osg::ref_ptr< osgGA::FlightManipulator > flight = new osgGA::FlightManipulator;
    flight->setHomePosition( osg::Vec3f(0.f,0.f,20.f), osg::Vec3f(0.f,0.f,20.f)+osg::Vec3f(0.f,1.f,0.f), osg::Vec3f(0,0,1) );
    viewer.setCameraManipulator( flight.get() );

    viewer.getCamera()->setViewMatrixAsLookAt( eye, centre, up    );

    viewer.addEventHandler( oceanSurface->getEventHandler() );
    viewer.addEventHandler( new osgGA::StateSetManipulator( viewer.getCamera()->getOrCreateStateSet() ) );

    viewer.setSceneData( oceanSurface.get() ); 

    viewer.realize();

    double eventAvg=0.0, updateAvg=0.0, cullAvg=0.0, gpuAvg=0.0;

    while(!viewer.done())
    {
        viewer.frame();    
    }

    return 0;
}
