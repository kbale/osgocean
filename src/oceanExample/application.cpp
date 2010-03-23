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

#include <string>
#include <vector>
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/StateSetManipulator>

#include <osg/Notify>

#include <osgDB/ReadFile>

#include <osgOcean/OceanScene>
#include <osgOcean/Version>
#include <osgOcean/ShaderManager>

#include "SceneEventHandler.h"
#include "SceneModel.h"
#include "TextHUD.h"

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
    viewer.addEventHandler( new osgGA::StateSetManipulator( viewer.getCamera()->getOrCreateStateSet() ) );

    osg::ref_ptr<TextHUD> hud = new TextHUD;

    osgOcean::ShaderManager::instance().enableShaders(!disableShaders);
    osg::ref_ptr<SceneModel> scene = new SceneModel(windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight, useVBO);
    
    if (disableShaders)
    {
        // Disable all special effects that depend on shaders.

        // These depend on fullscreen RTT passes and shaders to do their effects.
        scene->getOceanScene()->enableDistortion(false);
        scene->getOceanScene()->enableGlare(false);
        scene->getOceanScene()->enableUnderwaterDOF(false);

        // These are only implemented in the shader, with no fixed-pipeline equivalent
        scene->getOceanScene()->enableUnderwaterScattering(false);
        // For these two, we might be able to use projective texturing so it would
        // work on the fixed pipeline?
        scene->getOceanScene()->enableReflections(false);
        scene->getOceanScene()->enableRefractions(false);
        scene->getOceanScene()->enableGodRays(false);  // Could be done in fixed pipeline?
        scene->getOceanScene()->enableSilt(false);     // Could be done in fixed pipeline?
    }

    scene->getOceanScene()->setOceanSurfaceHeight(oceanSurfaceHeight);
    viewer.addEventHandler(scene->getOceanSceneEventHandler());
    viewer.addEventHandler(scene->getOceanSurface()->getEventHandler());

    viewer.addEventHandler( new SceneEventHandler(scene.get(), hud.get(), viewer ) );
    viewer.addEventHandler( new osgViewer::HelpHandler );
    viewer.getCamera()->setName("MainCamera");

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
        osgDB::Registry::instance()->getDataFilePathList().push_back("resources/boat");
        const std::string filename = "boat.3ds";
        osg::ref_ptr<osg::Node> boat = osgDB::readNodeFile(filename);

        if(boat.valid())
        {
            boat->setNodeMask( scene->getOceanScene()->getNormalSceneMask() | 
                scene->getOceanScene()->getReflectedSceneMask() | 
                scene->getOceanScene()->getRefractedSceneMask() );

            osg::ref_ptr<osg::MatrixTransform> boatTransform = new osg::MatrixTransform;
            boatTransform->addChild(boat.get());
            boatTransform->setMatrix(osg::Matrix::translate(osg::Vec3f(0.0f, 160.0f,0.0f)));
            boatTransform->setUpdateCallback( new BoatPositionCallback(scene->getOceanScene()) );

            scene->getOceanScene()->addChild(boatTransform.get());   
        }
        else
        {
            osg::notify(osg::WARN) << "testCollision flag ignored - Could not find: " << filename << std::endl;
        }
    }

    viewer.setSceneData( root );

    viewer.realize();

    while(!viewer.done())
    {
        viewer.frame();    
    }

    return 0;
}
