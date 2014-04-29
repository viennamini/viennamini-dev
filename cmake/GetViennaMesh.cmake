IF(DOWNLOAD)
  ExternalProject_Add(viennamesh
    PREFIX viennamesh
    GIT_REPOSITORY https://github.com/viennamesh/viennamesh-dev.git
    GIT_TAG master
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/viennamesh"
    CMAKE_ARGS -DBUILD_SHARED_LIBS=${BUILD_VIENNASTAR_SHARED} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DBUILD_PYVIENNAMESH=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_DOXYGEN_DOCS=OFF -DBUILD_MANUAL=OFF
      -DVIENNAMESH_WITH_TRIANGLE=ON -DVIENNAMESH_WITH_TETGEN=ON -DVIENNAMESH_WITH_NETGEN=ON -DVIENNAMESH_NETGEN_WITH_OPENCASCADE=OFF
    INSTALL_COMMAND ""
  )
  ExternalProject_Get_Property(viennamesh SOURCE_DIR)
  ExternalProject_Get_Property(viennamesh BINARY_DIR)
ELSE(DOWNLOAD)
  ExternalProject_Add(viennamesh
    PREFIX viennamesh
    SOURCE_DIR $ENV{VIENNAMESHPATH}
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/viennamesh"
    CMAKE_ARGS -DBUILD_SHARED_LIBS=${BUILD_VIENNASTAR_SHARED} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DBUILD_PYVIENNAMESH=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_DOXYGEN_DOCS=OFF -DBUILD_MANUAL=OFF
      -DVIENNAMESH_WITH_TRIANGLE=ON -DVIENNAMESH_WITH_TETGEN=ON -DVIENNAMESH_WITH_NETGEN=ON -DVIENNAMESH_NETGEN_WITH_OPENCASCADE=OFF
    INSTALL_COMMAND ""
  )
  ExternalProject_Get_Property(viennamesh SOURCE_DIR)
  ExternalProject_Get_Property(viennamesh BINARY_DIR)
ENDIF(DOWNLOAD)


SET(VIENNAMESH_INCLUDE_DIRS ${SOURCE_DIR})
SET(VIENNAMESH_INCLUDE_DIRS ${VIENNAMESH_INCLUDE_DIRS} ${SOURCE_DIR}/external)

IF(BUILD_SHARED)
  set(LIBSUFFIX ".so")
ELSE(BUILD_SHARED)
  set(LIBSUFFIX ".a")
ENDIF(BUILD_SHARED)

SET(VIENNAMESH_LIBRARIES ${BINARY_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}viennamesh${LIBSUFFIX})
