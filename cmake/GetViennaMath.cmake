set(VIENNAMATH_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/viennamath")
set(VIENNAMATH_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/viennamath/install")
set(VIENNAMATH_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${VIENNAMATH_INSTALL_DIR} -DBUILD_EXAMPLES=OFF -DENABLE_DIST=OFF)


ExternalProject_Add(viennamath
  PREFIX ${VIENNAMATH_PREFIX}
  GIT_REPOSITORY https://github.com/viennamath/viennamath-dev.git
  GIT_TAG master
  CMAKE_ARGS ${VIENNAMATH_CMAKE_ARGS}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)
