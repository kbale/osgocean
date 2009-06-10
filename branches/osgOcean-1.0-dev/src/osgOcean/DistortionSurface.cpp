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


#include <osgOcean/DistortionSurface>

using namespace osgOcean;

#define USE_LOCAL_SHADERS

DistortionSurface::DistortionSurface( void )
{}

DistortionSurface::DistortionSurface( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture )
{
	build(corner,dims,texture);
}

DistortionSurface::DistortionSurface( const DistortionSurface &copy, const osg::CopyOp &copyop ):
	osg::Geode(copy,copyop)
{
}

void DistortionSurface::build( const osg::Vec3f& corner, const osg::Vec2f& dims, osg::TextureRectangle* texture )
{
	osg::notify(osg::INFO) << "DistortionSurface::build()"<< std::endl;

	removeDrawables( 0, getNumDrawables() );

	osg::Geometry* geom = new ScreenAlignedQuad(corner,dims,texture);
	addDrawable(geom);

	osg::StateSet* ss = new osg::StateSet;

	osg::ref_ptr<osg::Program> program = createShader();

	if(program.valid())
		ss->setAttributeAndModes( program, osg::StateAttribute::ON );
	else
		osg::notify(osg::WARN) << "DistortionSurface::build() Invalid Shader"<< std::endl;
	
	ss->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON);

	ss->addUniform( new osg::Uniform( "uFrameBuffer",  0 ) );
	ss->addUniform( new osg::Uniform( "uFrequency",    2.f ) );
	ss->addUniform( new osg::Uniform( "uOffset",       0.f ) );
	ss->addUniform( new osg::Uniform( "uSpeed",        1.f ) );
	ss->addUniform( new osg::Uniform( "uScreenRes",    dims ) );

	setStateSet(ss);

	setUserData( new DistortionDataType(*this) );
	setUpdateCallback( new DistortionCallback );
}

osg::Program* DistortionSurface::createShader(void)
{
	osg::ref_ptr<osg::Shader> vShader = new osg::Shader( osg::Shader::VERTEX );
	vShader->setName("distortion_surface_fragment");

	osg::ref_ptr<osg::Shader> fShader = new osg::Shader( osg::Shader::FRAGMENT );
	fShader->setName("distortion_surface_vertex");

#ifdef USE_LOCAL_SHADERS

	char water_distortion_vertex[] = 
		"varying vec4 vEyePos;\n"
		"\n"
		"void main(void)\n"
		"{\n"
		"  gl_TexCoord[0] = gl_MultiTexCoord0;"
		"	vEyePos = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		"	gl_Position = ftransform();\n"
		"}\n";

	char water_distortion_fragment[] =
		"// Based on Jon Kennedy's heat haze shader\n"
		"// Copyright (c) 2002-2006 3Dlabs Inc. Ltd.\n"
		"\n"
		"uniform float uFrequency;\n"
		"uniform float uOffset;\n"
		"uniform float uSpeed;\n"
		"uniform vec2  uScreenRes;\n"
		"\n"
		"uniform samplerRect uFrameBuffer;\n"
		"\n"
		"varying vec4 vEyePos;\n"
		"\n"
		"void main (void)\n"
		"{\n"
		"	vec2 index;\n"
		"\n"
		"	// perform the div by w to put the texture into screen space\n"
		"	float recipW = 1.0 / vEyePos.w;\n"
		"	vec2 eye = vEyePos.xy * vec2(recipW);\n"
		"\n"
		"	float blend = max(1.0 - eye.y, 0.0);   \n"
		"	  \n"
		"	// calc the wobble\n"
		"	// index.s = eye.x ;\n"
		"	index.s = eye.x + blend * sin( uFrequency * 5.0 * eye.x + uOffset * uSpeed ) * 0.004;\n"
		"	index.t = eye.y + blend * sin( uFrequency * 5.0 * eye.y + uOffset * uSpeed ) * 0.004;\n"
		"	  \n"
		"	// scale and shift so we're in the range 0-1\n"
		"	index.s = index.s * 0.5 + 0.5;\n"
		"	index.t = index.t * 0.5 + 0.5;\n"
		"\n"
		"	vec2 recipRes = vec2(1.0/uScreenRes.x, 1.0/uScreenRes.y);\n"
		"\n"
		"	index.s = clamp(index.s, 0.0, 1.0 - recipRes.x);\n"
		"	index.t = clamp(index.t, 0.0, 1.0 - recipRes.y);\n"
		"\n"
		"	// scale the texture so we just see the rendered framebuffer\n"
		"	index.s = index.s * uScreenRes.x;\n"
		"	index.t = index.t * uScreenRes.y;\n"
		"	  \n"
		"	vec3 RefractionColor = vec3( textureRect( uFrameBuffer, index ) );\n"
		"\n"
		"	gl_FragColor = vec4( RefractionColor, 1.0 );\n"
		"	//gl_FragColor = textureRect( uFrameBuffer, gl_TexCoord[0].st );\n"
      "}\n";

	vShader->setShaderSource(water_distortion_vertex);
	fShader->setShaderSource(water_distortion_fragment);

#else
	if( !vShader->loadShaderSourceFromFile(water_distortion.vert") )
		return NULL;

	if( !fShader->loadShaderSourceFromFile(water_distortion.frag") )
		return NULL;
#endif

	osg::Program* program = new osg::Program;
	program->addShader( vShader.get() );
	program->addShader( fShader.get() );

	return program;
}

void DistortionSurface::update( const double& dt )
{
	static float val = 0.f;
	static float inc = 1.39624444f; //(2PI/4.5f;)

	val += inc * dt; 

	if(val >= 6.2831f) 
		val = 0.f;

	getOrCreateStateSet()->getOrCreateUniform("uOffset", osg::Uniform::FLOAT)->set(val);
}

// --------------------------------------------
//          Callback implementations
// --------------------------------------------

DistortionSurface::DistortionDataType::DistortionDataType(DistortionSurface& surface):
_surface( surface ),
_oldTime(0.0),
_newTime(0.0)
{}

void DistortionSurface::DistortionDataType::update( const double& time )
{
	_oldTime = _newTime;
	_newTime = time;

	_surface.update(_newTime-_oldTime);
}

void DistortionSurface::DistortionCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	osg::ref_ptr<DistortionSurface::DistortionDataType> data 
		= dynamic_cast<DistortionSurface::DistortionDataType*> ( node->getUserData() );

	if(data)
	{
		if(nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
		{
			data->update( nv->getFrameStamp()->getSimulationTime() );
		}
	}

	traverse(node, nv); 
}