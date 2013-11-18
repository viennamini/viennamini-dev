# - Find ViennaMaterials
#
# Defines the following if found:
#   VIENNAMATERIALS_FOUND        : TRUE if found, FALSE otherwise
#   VIENNAMATERIALS_INCLUDE_DIRS : Include directories 
#   VIENNAMATERIALS_LIBRARIES    : The libraries 
#
# Module looks for the path provided by the environment variable
#   VIENNAMATERIALSPATH


FIND_PATH(VIENNAMATERIALS_DIR 
          NAMES viennamaterials/forwards.h
          PATHS $ENV{VIENNAMATERIALSPATH}
          )

SET(VIENNAMATERIALS_INCLUDE_DIRS ${VIENNAMATERIALS_DIR}/viennamaterials)

find_package_handle_standard_args(ViennaMaterials DEFAULT_MSG VIENNAMATERIALS_DIR)

mark_as_advanced(VIENNAMATERIALS_DIR)


## 
SET(VIENNAMATERIALS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/viennamaterials")
SET(VIENNAMATERIALS_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/viennamaterials/install")
SET(VIENNAMATERIALS_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${VIENNAMATERIALS_INSTALL_DIR} -DBUILD_EXAMPLES=OFF -DBUILD_DTD_CHECK_SUPPORT=OFF -DBUILD_SERIALIZATION=OFF)

ExternalProject_Add(viennamaterials
  PREFIX ${VIENNAMATERIALS_PREFIX}
  SOURCE_DIR ${VIENNAMATERIALS_DIR}
  CMAKE_ARGS ${VIENNAMATERIALS_CMAKE_ARGS}
  INSTALL_COMMAND ""
)

SET(VIENNAMATERIALS_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/viennamaterials/src/viennamaterials-build/libviennamaterials.a)




