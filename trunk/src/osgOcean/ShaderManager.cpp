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

#include <osgOcean/ShaderManager>
#include <osgDB/ReadFile>

using namespace osgOcean;

ShaderManager::ShaderManager()
{
}

ShaderManager& ShaderManager::instance()
{
    static ShaderManager s_instance;
    return s_instance;
}

/** Get the value of a global definition that was previously set using
 *  setGlobalDefinition().
 */
std::string ShaderManager::getGlobalDefiniton(const std::string& name)
{
    GlobalDefinitions::const_iterator it = _globalDefinitions.find(name);
    if (it != _globalDefinitions.end())
        return it->second;

    return "";
}

/** Creates a shader program using either the given strings as shader 
 *  source directly, or as filenames to load the shaders from disk, 
 *  depending on the value of the \c loadFromFiles parameter.
 */
osg::Program* ShaderManager::createProgram( const std::string& name, 
                                            const std::string& vertexSrc, 
                                            const std::string& fragmentSrc, 
                                            bool loadFromFiles )
{
    osg::ref_ptr<osg::Shader> vShader = 0;
    osg::ref_ptr<osg::Shader> fShader = 0;
    
	 if (loadFromFiles)
    {
		  vShader = osgDB::readShaderFile(osg::Shader::VERTEX, vertexSrc);
        if (!vShader)
        {
            osg::notify(osg::WARN) << "Could not read shader from file " << vertexSrc << std::endl;
            return NULL;
        }

    	  fShader = osgDB::readShaderFile(osg::Shader::FRAGMENT, fragmentSrc);
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

    std::string globalDefinitionsList = buildGlobalDefinitionsList();
    vShader->setShaderSource(globalDefinitionsList + vShader->getShaderSource());
    fShader->setShaderSource(globalDefinitionsList + fShader->getShaderSource());

    vShader->setName(name+"_vertex_shader");
    fShader->setName(name+"_fragment_shader");

    osg::Program* program = new osg::Program;
    program->addShader( vShader );
    program->addShader( fShader );

    return program;
}

std::string ShaderManager::buildGlobalDefinitionsList()
{
    std::string list;
    for (GlobalDefinitions::const_iterator it = _globalDefinitions.begin();
         it != _globalDefinitions.end(); ++it)
    {
        list += "#define " + it->first + " " + it->second + "\n";
    }

    return list;
}
