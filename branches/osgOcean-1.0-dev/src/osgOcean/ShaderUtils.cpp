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

#include <osgOcean/ShaderUtils>
#include <osgDB/ReadFile>

/** Creates a shader program using either the given strings as shader 
 *  source directly, or as filenames to load the shaders from disk, 
 *  depending on the value of the \c loadFromFiles parameter.
 */
osg::Program* osgOcean::createProgram( const std::string& name, 
                             const std::string& vertexSrc, 
                             const std::string& fragmentSrc, 
                             bool loadFromFiles )
{
    osg::ref_ptr<osg::Shader> vShader = 0;
    osg::ref_ptr<osg::Shader> fShader = 0;
    if (loadFromFiles)
    {
        vShader = osgDB::readShaderFile(vertexSrc);
        if (!vShader)
        {
            osg::notify(osg::WARN) << "Could not read shader from file " << vertexSrc << std::endl;
            return NULL;
        }

    	fShader = osgDB::readShaderFile(fragmentSrc);
        if (!fShader)
        {
            osg::notify(osg::WARN) << "Could not read shader from file " << fragmentSrc << std::endl;
            return NULL;
        }
    }
    else
    {
        vShader = new osg::Shader( osg::Shader::VERTEX, vertexSrc );
        fShader = new osg::Shader( osg::Shader::FRAGMENT, fragmentSrc );
    }

    vShader->setName(name+"_vertex_shader");
    fShader->setName(name+"_fragment_shader");

    osg::Program* program = new osg::Program;
    program->addShader( vShader );
    program->addShader( fShader );

    return program;
}

