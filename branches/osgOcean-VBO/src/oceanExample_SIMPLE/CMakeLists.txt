SET(HEADER_PATH ${osgOcean_SOURCE_DIR}/include/OceanExample )
SET(LIB_HEADER_PATH ${osgOcean_SOURCE_DIR}/include )

SET( EXAMPLE_SHADERS
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain.vert
  ${osgOcean_SOURCE_DIR}/resources/shaders/terrain.frag
)

INCLUDE_DIRECTORIES(
   ${LIB_HEADER_PATH}
   ${HEADER_PATH}
)

ADD_EXECUTABLE(

  oceanExample_SIMPLE

  application.cpp
)

TARGET_LINK_LIBRARIES(
  oceanExample_SIMPLE

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

INSTALL(
   TARGETS oceanExample_SIMPLE
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

INSTALL(
  FILES ${EXAMPLE_SHADERS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/resources/shaders
)

