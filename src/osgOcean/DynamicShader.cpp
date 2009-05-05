#include <osgOcean/DynamicShader>

using namespace osgOcean;

DynamicShader::DynamicShader( void ):
	_vShader( new osg::Shader(osg::Shader::VERTEX) ),
	_fShader( new osg::Shader(osg::Shader::FRAGMENT) ),
	_program( new osg::Program )
{
	_vShader->setName("Vertex_Shader");
	_fShader->setName("Fragment_Shader");
}

DynamicShader::DynamicShader( const std::string& vertexPath, 
									   const std::string& fragmentPath ):
	_vShader( new osg::Shader(osg::Shader::VERTEX) ),
	_fShader( new osg::Shader(osg::Shader::FRAGMENT) ),
	_program( new osg::Program )
{
	_vShader->setName("Vertex_Shader");
	_fShader->setName("Fragment_Shader");
	load( vertexPath, fragmentPath );
}

void DynamicShader::load( const std::string& vertexPath, const std::string& fragmentPath )
{
	storeSource( vertexPath,	_origVertexSource );
	storeSource( fragmentPath, _origFragmentSource );
	
	rebuild();
}

void DynamicShader::rebuild( void )
{
	amendShader( _newVertexSource,	_origVertexSource );
	amendShader( _newFragmentSource, _origFragmentSource );

	_vShader->setShaderSource( _newVertexSource );
	_fShader->setShaderSource( _newFragmentSource );

	_vShader->dirtyShader();
	_fShader->dirtyShader();

	_program->addShader( _vShader.get() );
	_program->addShader( _fShader.get() );
}

bool DynamicShader::storeSource( const std::string& path, std::string& source )
{
	std::stringstream stream(std::stringstream::in);

	std::ifstream fileStream( path.c_str(), std::ios::in );

	if( !fileStream ){
		osg::notify(osg::WARN) << "ERROR: Could not open file " << path << std::endl;
		return false;
	}

	while( !fileStream.eof() )
	{
		char line[1024];
		fileStream.getline(line, 1024);
		stream << line << std::endl;
	}

	source = stream.str();

	return true;
}

void DynamicShader::addDefine( const std::string& name )
{
	// check if it already exists
	for(OPTION_LIST::iterator it = _optionList.begin(); 
		it != _optionList.end(); 
		++it)
	{
		if(name.compare(*it)==0)
			return;
	}
	// add to list
	_optionList.push_back( name );
}

void DynamicShader::addDefine( const std::string& name, unsigned int val )
{
	std::stringstream ss( std::stringstream::in );
	ss << name << " " << val;

	for(OPTION_LIST::iterator it = _optionList.begin(); 
		it != _optionList.end(); 
		++it)
	{
		if(name.compare(*it)==0)
			return;
	}

	_optionList.push_back( ss.str() );
	
}

void DynamicShader::amendShader( std::string& newSource, const std::string& oldSource )
{
	std::stringstream ss(std::stringstream::in);

	for( OPTION_LIST::iterator it=_optionList.begin(); 
		  it!=_optionList.end();
		  ++it )
	{
		ss << "#define " << *it << "\n;";
	}

	ss << oldSource;

	newSource = ss.str();
}

void DynamicShader::printShader(void)
{
	osg::notify(osg::NOTICE) << "VERTEX SHADER\n" << std::endl;
	osg::notify(osg::NOTICE) << _newVertexSource << std::endl;

	osg::notify(osg::NOTICE) << "FRAGMENT SHADER\n" << std::endl;
	osg::notify(osg::NOTICE) << _newFragmentSource << std::endl;
}