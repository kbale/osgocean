# Find double-precision (double) version of FFTW3

FIND_PATH(
	FFTW3-3_INCLUDE_DIR
	NAMES fftw3.h
	/usr/local/include
    /usr/include
)

FIND_LIBRARY(
	FFTW3-3_LIBRARY
	NAMES fftw3-3 libfftw3-3
	/usr/local/lib
    /usr/lib
)

SET(FFTW3_FOUND "NO")

IF( FFTW3-3_INCLUDE_DIR AND FFTW3-3_LIBRARY )
    SET(FFTW3_FOUND "YES")
ENDIF()
