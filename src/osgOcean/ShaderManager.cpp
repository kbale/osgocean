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
#include <osgDB/fileutils>
#include <osg/Version>

using namespace osgOcean;


osg::Shader* readShader(const std::string& filename)
{
    // The .vert and .frag extensions were added to the GLSL plugin in OSG 
    // 2.7.3, and the automatic setting of shader type depending on the 
    // extension was added in OSG 2.9.1. The code below lets us use OSG
    // 2.6 and still get the same behavior.
#if OPENSCENEGRAPH_MAJOR_VERSION > 2 || \
    (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION > 9) || \
    (OPENSCENEGRAPH_MAJOR_VERSION == 2 && OPENSCENEGRAPH_MINOR_VERSION == 9 && OPENSCENEGRAPH_PATCH_VERSION >= 1)

    // This will search the registry's file path.
    return osgDB::readShaderFile(filename);

#else

    // Determine shader type from the extension
    osg::Shader::Type type = osg::Shader::UNDEFINED;
    if (filename.find("vert") == filename.length() - 4)
        type = osg::Shader::VERTEX;
    else if (filename.find("frag") == filename.length() - 4)
        type = osg::Shader::FRAGMENT;
    else
        return 0;

    // Find the shader file in the osgDB data path list.
    std::string fullpath = osgDB::findDataFile(filename);
    if (fullpath.empty())
        return 0;

    // Read the shader file.
    osg::Shader* shader = osg::Shader::readShaderFile(type, fullpath);
    return shader;

#endif
}

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
        vShader = readShader(vertexSrc);
        if (!vShader)
        {
            osg::notify(osg::WARN) << "Could not read shader from file " << vertexSrc << std::endl;
            return NULL;
        }

        fShader = readShader(fragmentSrc);
        if (!fShader)
        {
            osg::notify(osg::WARN) << "Could not read shader from file " << fragmentSrc << std::endl;
            return NULL;
        }
    }
    else
    {
        if (!vertexSrc.empty())
        {
            vShader = new osg::Shader( osg::Shader::VERTEX, vertexSrc );
        }
        if (!fragmentSrc.empty())
        {
            fShader = new osg::Shader( osg::Shader::FRAGMENT, fragmentSrc );
        }
    }

    osg::Program* program = new osg::Program;
    program->setName(name);

    std::string globalDefinitionsList = buildGlobalDefinitionsList(name);
    if (vShader.valid())
    {
        vShader->setShaderSource(globalDefinitionsList + vShader->getShaderSource());
        vShader->setName(name+"_vertex_shader");
        program->addShader( vShader.get() );
    }
    if (fShader.valid())
    {
        fShader->setShaderSource(globalDefinitionsList + fShader->getShaderSource());
        fShader->setName(name+"_fragment_shader");
        program->addShader( fShader.get() );
    }

    return program;
}

std::string ShaderManager::buildGlobalDefinitionsList(const std::string& name)
{
    std::string list;

    if (!name.empty())
    {
        list += "// " + name + "\n";
    }
   // list += "#version 120\n";

    for (GlobalDefinitions::const_iterator it = _globalDefinitions.begin();
         it != _globalDefinitions.end(); ++it)
    {
        list += "#define " + it->first + " " + it->second + "\n";
    }

    return list;
}
