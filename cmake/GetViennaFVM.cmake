set(VIENNAFVM_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/viennafvm")
set(VIENNAFVM_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/viennafvm/install")
set(VIENNAFVM_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${VIENNAFVM_INSTALL_DIR} -DBUILD_EXAMPLES=OFF )


ExternalProject_Add(viennafvm
  PREFIX ${VIENNAFVM_PREFIX}
  GIT_REPOSITORY https://github.com/viennafvm/viennafvm-dev.git
  GIT_TAG master
  CMAKE_ARGS ${VIENNAFVM_CMAKE_ARGS}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)