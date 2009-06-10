osgOcean v1.0
-------------

In order to run the oceanExample, you will need the resources which
can be found here:
http://code.google.com/p/osgocean/downloads/list

The library looks for shaders in the following directory:
resources/shaders/

Likewise with textures it will look for them in
resources/textures/

You should therefore make sure that resource directory is either
setup as a path variable or in the same path as the DLL or example
executable.

osgOcean also requires an FFT generator library. It can work with either
FFTW or FFTSS. FFTW is GPL, so compiling osgOcean with it will make that build of
osgOcean GPL as well, whereas FFTSS is LGPL which will allow osgOcean to
remain LGPL.

FFTW:  http://www.fftw.org/
FFTSS: http://www.ssisc.org/fftss/
