FIND_PATH( 
	FFTSS_INCLUDE_DIR 
	NAMES fftw3compat.h
	/usr/local/include
    /usr/include
)

FIND_LIBRARY(
	FFTSS_LIBRARY
	NAMES fftss libfftss
	/usr/local/lib
    /usr/lib
)

SET(FFTSS_FOUND "NO")

IF( FFTSS_INCLUDE_DIR AND FFTSS_LIBRARY )
    SET(FFTSS_FOUND "YES")
ENDIF(FFTSS_INCLUDE_DIR AND FFTSS_LIBRARY-3_LIBRARY)
