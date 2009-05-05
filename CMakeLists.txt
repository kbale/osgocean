# The name of our project is "osgOcean". CMakeLists files in this project can
# refer to the root source directory of the project as ${osgOcean_SOURCE_DIR}
# to the root binary directory of the project as ${osgOcean_BINARY_DIR}.

cmake_minimum_required (VERSION 2.6)

project (osgOcean)

SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${osgOcean_BINARY_DIR}/bin)
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${osgOcean_BINARY_DIR}/lib)
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${osgOcean_BINARY_DIR}/lib)

SET( CMAKE_USE_RELATIVE_PATHS true )

set( CMAKE_DEBUG_POSTFIX D )

set(CMAKE_MODULE_PATH "${osgOcean_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")

find_package (osg REQUIRED)
include_directories (${OSG_INCLUDE_DIR})

find_package (osgDB REQUIRED)
include_directories (${OSGDB_INCLUDE_DIR})

find_package (osgGA REQUIRED)
include_directories (${OSGGA_INCLUDE_DIR})

find_package (osgViewer REQUIRED)
include_directories (${OSGVIEWER_INCLUDE_DIR})

find_package (osgUtil REQUIRED)
include_directories (${OSGUTIL_INCLUDE_DIR})

find_package (osgText REQUIRED)
include_directories (${OSGTEXT_INCLUDE_DIR})

find_package (OpenThreads REQUIRED)
include_directories(${OPENTHREADS_INCLUDE_DIR})

find_package (fftw3f-3 REQUIRED)
include_directories (${FFTW3F-3_INCLUDE_DIR})


add_subdirectory (src)
