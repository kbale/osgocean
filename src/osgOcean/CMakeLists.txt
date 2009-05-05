SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/osgOcean )

SET( SHADER_FILES
  ${osgOcean_SOURCE_DIR}/resources/shaders/water.f
  ${osgOcean_SOURCE_DIR}/resources/shaders/water.v
  
)

SET( TEXTURE_FILES
  ${osgOcean_SOURCE_DIR}/resources/textures/sea_foam.png
)

SET( LIB_HEADERS

  ${HEADER_PATH}/DynamicShader
  ${HEADER_PATH}/FFTOceanSurface
  ${HEADER_PATH}/FFTSimulation
  ${HEADER_PATH}/MipmapGeometry
  ${HEADER_PATH}/OceanScene
  ${HEADER_PATH}/OceanTechnique
  ${HEADER_PATH}/OceanTile
  ${HEADER_PATH}/Export
)

ADD_DEFINITIONS(-DOSGOCEAN_LIBRARY)

include_directories (
   ${OSG_INCLUDE_DIR}
   ${FFTW3F-3_INCLUDE_DIR}
   ${osgOcean_SOURCE_DIR}/include/ 
)

add_library(
  osgOcean
  SHARED
  ${LIB_HEADERS}
  DynamicShader.cpp
  FFTOceanSurface.cpp
  FFTSimulation.cpp
  MipmapGeometry.cpp
  OceanScene.cpp
  OceanTechnique.cpp
  OceanTile.cpp
)

set_target_properties(
  osgOcean
  PROPERTIES FRAMEWORK TRUE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

target_link_libraries(
  osgOcean

  optimized ${OSG_LIBRARY}
  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSGUTIL_LIBRARY}
  debug ${OSGUTIL_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}
  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OPENTHREADS_LIBRARY}
  debug ${OPENTHREADS_LIBRARY_DEBUG}

  ${FFTW3F-3_LIBRARY}
)

install(
  TARGETS osgOcean
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  
)

install(
  FILES ${SHADER_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/shaders
)

install(
  FILES ${TEXTURE_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/textures
)
