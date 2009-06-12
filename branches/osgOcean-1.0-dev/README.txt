osgOcean v1.0
-------------

DEPENDANCIES
------------

Resources
---------

Both the library and the example require a set of resource
files (models/textures) which can be downloaded from here:
http://code.google.com/p/osgocean/downloads/list

osgOcean uses the osgDB registry to find the resource files.
By default it adds the following paths to the registry:

Shader path:
resources/shaders/

Texture path:
resources/textures/

If you wish to move these resources you must update the data file path 
list within the registry yourself. This can be down from outside the 
library with the following code:

osgDB::FilePathList& pathList = osgDB::Registry::instance()->getDataFilePathList();
pathList.push_back( new_path );

Libraries
---------

osgOcean also requires a Fast Fourier Transform library. It can work 
with either FFTW or FFTSS. 

**IMPORTANT LICENSE ISSUE**
FFTW is released under a General Public License, by selecting this 
option in CMAKE the resulting build of osgOcean will also be covered under 
the GPL license.

GPL License details: http://www.gnu.org/copyleft/gpl.html

FFTSS is covered under a Lesser General Public License which allows 
the osgOcean library to remain LGPL. 

LGPL License details: http://www.gnu.org/licenses/lgpl.html or see LICENSE.txt

FFTW and FFTSS can be downloaded from:

FFTW:  http://www.fftw.org/
FFTSS: http://www.ssisc.org/fftss/
