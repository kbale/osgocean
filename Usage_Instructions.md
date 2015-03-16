# Using the library #

## Compiling ##

osgOcean 1.0.1 is a cross platform library.

Use CMake version 2.6.3 or greater to generate native makefiles or workspaces for your chosen compiler environment.

<a href='http://www.cmake.org/'><a href='http://www.cmake.org/'>http://www.cmake.org/</a></a>

## Dependencies ##

### OpenSceneGraph ###

The main dependancy is of course the OpenSceneGraph libraries. osgOcean 1.0.1 was built with with OSG 2.8.2 in mind, but it supports OSG 2.6 and might even run on earlier versions (untested). The libraries can be downloaded here:

<a href='http://www.openscenegraph.org/projects/osg/wiki/Downloads'> <a href='http://www.openscenegraph.org/projects/osg/wiki/Downloads'>http://www.openscenegraph.org/projects/osg/wiki/Downloads</a> </a>

### Fast Fourier Transform Library ###

osgOcean needs to be linked against a Fast Fourier Transform library. You can choose either the FFTW or FFTSS library.

<a href='http://www.fftw.org/'>FFTW</a> is GPL, so linking osgOcean to it will make osgOcean GPL (and your application too if you link it to this version of osgOcean).

<a href='http://www.ssisc.org/fftss/'>FFTSS</a> is LGPL, so linking osgOcean to it means that osgOcean retains its LGPL license, and thus it can be used in conformance with the LGPL license.

You will need to specify the include directory and library file of your installed and compiled FFT library when building the project. FFTW offers precompiled binaries for Windows, though it requires that you generate the import libraries before being able to use it (see <a href='http://www.fftw.org/install/windows.html'><a href='http://www.fftw.org/install/windows.html'>http://www.fftw.org/install/windows.html</a></a>). FFTSS needs to be compiled, but it is painless as VC7 project files are provided in the win32 subdirectory of the source archive. You only need to compile the release version.

## Building osgOcean ##

### Windows ###

On Windows, you run the CMake GUI. At the top you will have a field "Where is the source code ?", point that to the base osgOcean directory (the one that contains src/, include/, resources/, etc.). Then in "Where to build the binaries?", I suggest you point it to a build/ subdirectory  under that same osgOcean directory (which you will create). Then click Configure, fill in any paths that are missing (FFTSS and OSG include and library paths probably), set CMAKE\_INSTALL\_PREFIX to for example your osgOcean base directory, click Configure again and then (if there is no error) Generate.

Then you can open the generated osgOcean.sln file (which will be in build/) in Visual Studio and build it, and then run the oceanExample.

### Linux ###

On Linux, do this:

cd <the osgOcean base directory>
mkdir build
cd build
ccmake ..

Then press C for configure, fill in any paths that are missing (FFTSS and OSG include and library paths probably), set CMAKE\_INSTALL\_PREFIX to for example your osgOcean base directory, press C again and then (if there is no error) G for generate.

Then you can run make from the build directory and it will compile osgOcean. You can run make install, that will copy the binaries into osgOcean/bin and osgOcean/lib. Then to run the oceanExample you'd have to add osgOcean/lib to your LD\_LIBRARY\_PATH (as well as your osg lib directory).

## Resources ##

Both the library and the example require a set of resource
files (models/textures) which can be downloaded from here:

<a href='http://osgocean.googlecode.com/files/osgOcean-Resources-1.0.1.rar'><a href='http://osgocean.googlecode.com/files/osgOcean-Resources-1.0.1.rar'>http://osgocean.googlecode.com/files/osgOcean-Resources-1.0.1.rar</a></a>

Once you've downloaded them, extract the 'Island','Textures' and 'Boat' folders into the resources directory found in the root of the source code tree. The install project will copy the relevant data files to the bin path like so:

```
%install_path%/bin/resources/
```

osgOcean uses the osgDB registry to find the resource files.
By default it adds the following paths to the registry:

Shader path:
resources/shaders/

Texture path:
resources/textures/

If you wish to move these resources you must update the data file path
list within the registry yourself. This can be down from outside the
library with the following code:

```
osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();
pathList.push_back( new_path );
```

## Interaction with other techniques / nodekits ##

In theory, any technique that uses multiple passes could potentially conflict with osgOcean's own multiple passes.

osgOcean has been successfully used with osgShadow's shadow algorithms, but only if most post-render effects (DOF and glare) are turned off. The osgOcean::OceanScene node should be placed under the ShadowedScene, with the rest of the scene under that. OceanScene will detect the shadow pass using the camera names "ShadowCamera" and "AnalysisCamera", and will act as a normal Group node in that case. If you write your own shadow technique and want to use it with osgOcean, it should use this name for the shadow pass camera as well. Also make sure the shadow pass camera is set as PRE\_RENDER with an index <= 0, so that the reflection/refraction passes happen after it (if they happened before, they would use the shadows from the previous frame).

osgOcean has been used with osgPPU (specifically the HDR effect as implemented in osgPPU's HDR example), but again with most osgOcean effects turned off. I have not yet confirmed which effects would work, again we only turned on reflection.

Another problem arises when using osgOcean with a CompositeViewer with multiple views. The last view added to the viewer (so presumably the last one to be rendered) will control LOD of the ocean tiles, and some effects will have problems/glitches when used in multiple views. Work is underway to make the effects work in multiple views - for now reflection, refraction and height map effects should work. I have modified the oceanExample to demonstrate this when run with the --compositeViewer argument. The post-render passes mentioned above (DOF, glare) won't work at all in multiple views, I get a black screen. There is also a method in OceanScene to disable the reflection/refraction/height map effects in specific views. This will help performance when some views don't need these effects.

## Bugs / Suggestions ##

Please add any bugs or suggestions you have to the <a href='http://code.google.com/p/osgocean/issues/list'>issue tracker</a> and I'll try to address them.