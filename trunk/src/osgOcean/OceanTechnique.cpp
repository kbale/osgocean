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

#include <osgOcean/OceanTechnique>

using namespace osgOcean;

OceanTechnique::OceanTechnique(void):
    _isDirty    ( true ),
    _isAnimating( true )
{

}

OceanTechnique::OceanTechnique( const OceanTechnique& copy, const osg::CopyOp& copyop ):
    osg::Geode  ( copy, copyop ),
    _isDirty    ( true ),
    _isAnimating( copy._isAnimating )
{

}

void OceanTechnique::build(void)
{
    osg::notify(osg::DEBUG_INFO) << "OceanTechnique::build() Not Implemented" << std::endl;
}

float OceanTechnique::getSurfaceHeight(void) const
{
    osg::notify(osg::DEBUG_INFO) << "OceanTechnique::getSurfaceHeight() Not Implemented" << std::endl;
    return 0.f;
}

/** Check if the ocean surface is visible or not. Basic test is very 
    simple, subclasses can do a better test. */
bool OceanTechnique::isVisible( osgUtil::CullVisitor& cv, bool eyeAboveWater )
{
    if (getNodeMask() == 0) return false;

    // Use a small cutoff to unconditionally cull ocean surface.
    // This assumes the view frustum is 45 degrees wide...
    static const float cutoff = osg::PI/8;      // 45 degrees divided by 2
    osg::Vec3 lookVector = cv.getLookVectorLocal();
    float dotProduct = lookVector * osg::Vec3(0,0,1);
    return ( eyeAboveWater && dotProduct <  cutoff) ||
           (!eyeAboveWater && dotProduct > -cutoff);

    // A better way would be to check if any of the frustum corners intersect 
    // the plane at (0,0,ocean_height) with normal (0,0,1), and if not then 
    // return true.
}


OceanTechnique::EventHandler::EventHandler(OceanTechnique* oceanSurface):
_oceanSurface(oceanSurface)
{
}

bool OceanTechnique::EventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*)
{
    if (ea.getHandled()) return false;

    // Nothing to do

    return false;
}

/** Get the keyboard and mouse usage of this manipulator.*/
void OceanTechnique::EventHandler::getUsage(osg::ApplicationUsage& usage) const
{
}
