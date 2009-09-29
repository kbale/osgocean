SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/osgOcean )

SET( SHADER_FILES
  ${osgOcean_SOURCE_DIR}/resources/shaders/water.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/water.vert

  ${osgOcean_SOURCE_DIR}/resources/shaders/godrays.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/godrays.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/godray_screen_blend.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/godray_screen_blend.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/godray_glare.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/godray_glare.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/silt_quads.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/silt_quads.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/silt_points.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/silt_points.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/streak.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/streak.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/glare_composite.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/glare_composite.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/downsample_glare.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/downsample.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/downsample.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/gaussian1.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/gaussian1.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/gaussian2.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/gaussian2.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/dof_combiner.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/dof_combiner.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/water_distortion.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/water_distortion.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/default_scene.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/default_scene.frag
)


SET( TEXTURE_FILES
  ${osgOcean_SOURCE_DIR}/resources/textures/sea_foam.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sun_glare.png
)

SET( LIB_HEADERS
  ${HEADER_PATH}/Cylinder
  ${HEADER_PATH}/DistortionSurface
  ${HEADER_PATH}/FFTOceanSurface
  ${HEADER_PATH}/FFTSimulation
  ${HEADER_PATH}/GodRays
  ${HEADER_PATH}/GodRayBlendSurface
  ${HEADER_PATH}/MipmapGeometry
  ${HEADER_PATH}/OceanScene
  ${HEADER_PATH}/OceanTechnique
  ${HEADER_PATH}/OceanTile
  ${HEADER_PATH}/RandUtils
  ${HEADER_PATH}/ScreenAlignedQuad
  ${HEADER_PATH}/ShaderManager
  ${HEADER_PATH}/SiltEffect
  ${HEADER_PATH}/WaterTrochoids
  ${HEADER_PATH}/Export
)

# Adds headers to the "Header Files" group in visual studio
SOURCE_GROUP( "Header Files" ${HEADER_PATH}/*.* )

ADD_DEFINITIONS(-DOSGOCEAN_LIBRARY)

INCLUDE_DIRECTORIES (
   ${OSG_INCLUDE_DIR}
   ${FFT_INCLUDE_DIR}
   ${osgOcean_SOURCE_DIR}/include/
)

ADD_LIBRARY(
  osgOcean
  SHARED
  ${LIB_HEADERS}
  Cylinder.cpp
  DistortionSurface.cpp
  FFTOceanSurface.cpp
  FFTSimulation.cpp
  GodRays.cpp
  GodRayBlendSurface.cpp
  MipmapGeometry.cpp
  OceanScene.cpp
  OceanTechnique.cpp
  OceanTile.cpp
  ScreenAlignedQuad.cpp
  ShaderManager.cpp
  SiltEffect.cpp
  WaterTrochoids.cpp
)

SET_TARGET_PROPERTIES(
  osgOcean
  PROPERTIES FRAMEWORK TRUE
  PUBLIC_HEADER "${LIB_HEADERS}"
)

TARGET_LINK_LIBRARIES(
  osgOcean

  optimized ${OSG_LIBRARY}
  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSGUTIL_LIBRARY}
  debug ${OSGUTIL_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}
  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OSGGA_LIBRARY}
  debug ${OSGGA_LIBRARY_DEBUG}
  optimized ${OPENTHREADS_LIBRARY}
  debug ${OPENTHREADS_LIBRARY_DEBUG}

  ${OPENGL_LIBRARY}
  ${FFT_LIBRARY}
)

INSTALL(
  TARGETS osgOcean
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

INSTALL(
  FILES ${SHADER_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/shaders
)

INSTALL(
  FILES ${TEXTURE_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/textures
)
