SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/osgOcean )

SET( SHADER_FILES
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_surface_vbo.vert

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godrays.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godrays.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godray_screen_blend.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godray_screen_blend.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godray_glare.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_godray_glare.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_silt_quads.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_silt_quads.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_silt_points.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_silt_points.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_streak.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_streak.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_glare_composite.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_glare_composite.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_downsample_glare.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_downsample.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_downsample.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_gaussian.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_gaussian1.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_gaussian2.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_dof_combiner.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_dof_combiner.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_water_distortion.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_water_distortion.frag

  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene.frag
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene_lispsm.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_ocean_scene_lispsm.frag
  
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_heightmap.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/osgOcean_heightmap.frag
)


SET( TEXTURE_FILES
  ${osgOcean_SOURCE_DIR}/resources/textures/sea_foam.png
  ${osgOcean_SOURCE_DIR}/resources/textures/sun_glare.png
)

SET( LIB_HEADERS
  ${HEADER_PATH}/Cylinder
  ${HEADER_PATH}/DistortionSurface
  ${HEADER_PATH}/FFTOceanTechnique
  ${HEADER_PATH}/FFTOceanSurface
  ${HEADER_PATH}/FFTOceanSurfaceVBO
  ${HEADER_PATH}/FFTSimulation
  ${HEADER_PATH}/GodRays
  ${HEADER_PATH}/GodRayBlendSurface
  ${HEADER_PATH}/MipmapGeometry
  ${HEADER_PATH}/MipmapGeometryVBO
  ${HEADER_PATH}/OceanScene
  ${HEADER_PATH}/OceanTechnique
  ${HEADER_PATH}/OceanTile
  ${HEADER_PATH}/RandUtils
  ${HEADER_PATH}/ScreenAlignedQuad
  ${HEADER_PATH}/ShaderManager
  ${HEADER_PATH}/SiltEffect
  ${HEADER_PATH}/WaterTrochoids
  ${HEADER_PATH}/Export
  ${HEADER_PATH}/Version
)

# Adds headers to the "Header Files" group in visual studio
SOURCE_GROUP( "Header Files" ${HEADER_PATH}/*.* )

# Adds shaders to the "Shaders" group in visual studio
SOURCE_GROUP( "Shaders" FILES ${SHADER_FILES} )

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
  FFTOceanTechnique.cpp
  FFTOceanSurface.cpp
  FFTOceanSurfaceVBO.cpp
  FFTSimulation.cpp
  GodRays.cpp
  GodRayBlendSurface.cpp
  MipmapGeometry.cpp
  MipmapGeometryVBO.cpp
  OceanScene.cpp
  OceanTechnique.cpp
  OceanTile.cpp
  ScreenAlignedQuad.cpp
  ShaderManager.cpp
  SiltEffect.cpp
  WaterTrochoids.cpp
  Version.cpp
  ${SHADER_FILES}
)

SET_TARGET_PROPERTIES(
  osgOcean
  PROPERTIES
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
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/osgOcean
)

INSTALL(
  FILES ${SHADER_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/shaders
)

INSTALL(
  FILES ${TEXTURE_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/textures
)
