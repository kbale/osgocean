SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/OceanExample )
SET(LIB_HEADER_PATH ${osgOcean_SOURCE_DIR}/include )

SET( SKYBOX_TEXTURES_1
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/down.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/east.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/north.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/south.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/up.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_clear/west.bmp
)

SET( SKYBOX_TEXTURES_2
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/down.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/east.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/north.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/south.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/up.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_dusk/west.bmp
)

SET( SKYBOX_TEXTURES_3
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/down.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/east.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/north.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/south.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/up.bmp
  ${osgOcean_SOURCE_DIR}/resources/textures/sky_fair_cloudy/west.bmp
)

include_directories( 
   ${LIB_HEADER_PATH} 
   ${HEADER_PATH}
   ${LIBFFTW3F-3_INCLUDE_DIR}
)

add_executable(

  oceanExample

  application.cpp
  SkyDome.cpp
  SphereSegment.cpp

  ${HEADER_PATH}/SkyDome.h
  ${HEADER_PATH}/SphereSegment.h
)

target_link_libraries(
  oceanExample

  osgOcean

  debug ${OSG_LIBRARY_DEBUG}
  optimized ${OSG_LIBRARY}

  debug ${OSGGA_LIBRARY_DEBUG}
  optimized ${OSGGA_LIBRARY}

  debug ${OSGVIEWER_LIBRARY_DEBUG}
  optimized ${OSGVIEWER_LIBRARY}

  debug ${OSGDB_LIBRARY_DEBUG}
  optimized ${OSGDB_LIBRARY}
  
  debug ${OSGTEXT_LIBRARY_DEBUG}
  optimized ${OSGTEXT_LIBRARY}
)

install(
   TARGETS oceanExample
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(
  FILES ${SKYBOX_TEXTURES_1}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/textures/sky_clear
)

install(
  FILES ${SKYBOX_TEXTURES_2}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/textures/sky_dusk
)

install(
  FILES ${SKYBOX_TEXTURES_3}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/textures/sky_fair_cloudy
)


