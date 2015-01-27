PROJECT(ViennaMini C CXX)

include(ExternalProject)
include(FindPackageHandleStandardArgs)

# CMake 2.8.2 is required as a minimum due to the
# ExternalProject_Add mechanism supporting GIT repositories
cmake_minimum_required(VERSION 2.8.2)

OPTION(DOWNLOAD           "Download external dependencies automatically"      OFF)
OPTION(BUILD_EXAMPLES     "Build the examples"                                OFF)
OPTION(VERBOSE            "Verbose messaging"                                 ON )
OPTION(USE_UNITS          "Use physical unit checks and conversions"          ON )
OPTION(BUILD_SHARED_LIBS  "Build all libraries as shared (dynamic) libraries" ON )
#OPTION(BUILD_TESTS "Build the tests" OFF)
#SET(PARALLEL_MODE  "Serial" CACHE STRING "Specify execution mode for linear algebra kernels, supported values are: Serial,OpenCL,OpenMP")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")




#set default build type
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

#set compiler flags
SET(CMAKE_CXX_FLAGS "-Wall -pedantic -Wno-unused-local-typedefs")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g ")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 ")
SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")

IF ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  IF    (GCC_VERSION VERSION_EQUAL 4.6)
    SET(CXX11_FLAG "-std=c++0x")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")
    message(STATUS "GCC C++ compiler supports C++11 using flag: ${CXX11_FLAG}")
  ELSEIF(GCC_VERSION VERSION_GREATER 4.6)
    SET(CXX11_FLAG "-std=c++11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")
    message(STATUS "GCC C++ compiler supports C++11 using flag: ${CXX11_FLAG}")
  ELSE  ()
    MESSAGE(FATAL_ERROR "Required C++11 features are not supported by this GCC compiler, required: >=4.6")
  ENDIF ()
#ELSEIF ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
#  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
#  message(STATUS "Detected Clang Compiler ${CLANG_VERSION}")
#  IF    (CLANG_VERSION VERSION_EQUAL 3.1)
#    SET(CXX11_FLAG "-std=c++11")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")
#    message(STATUS "Clang C++ compiler supports C++11 using flag: ${CXX11_FLAG}")
#  ELSE  ()
#    MESSAGE(FATAL_ERROR "Required C++11 features are not supported by this Clang compiler, required: >=3.1")
#  ENDIF ()
#ELSEIF ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
#  IF    (INTEL_VERSION VERSION_EQUAL 13.0)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#  ELSE  ()
#    MESSAGE(FATAL_ERROR "Required C++11 features are not supported by this Intel compiler, required: >=13.0")
#  ENDIF ()
ELSE ()
  MESSAGE(FATAL_ERROR "Unknown compiler detected, currently supported: GCC, Clang, Intel")
ENDIF ()


IF(VERBOSE)
  ADD_DEFINITIONS(-DVIENNAMINI_VERBOSE)
ENDIF(VERBOSE)

#STRING(TOLOWER "${PARALLEL_MODE}" PARALLEL_MODE) # make case unique
#IF(PARALLEL_MODE STREQUAL "serial")
#  MESSAGE(STATUS "Using serial linear algebra execution mode")
#ELSEIF(PARALLEL_MODE STREQUAL "opencl")
#  MESSAGE(STATUS "Using OpenCL linear algebra execution mode")
#  FIND_PACKAGE(OpenCL REQUIRED)
#  IF(OPENCL_FOUND)
#    INCLUDE_DIRECTORIES(${OPENCL_INCLUDE_DIRS})
#    SET(LIBRARIES ${LIBRARIES} ${OPENCL_LIBRARIES})
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DVIENNACL_WITH_OPENCL")
#  ENDIF(OPENCL_FOUND)
#ELSEIF(PARALLEL_MODE STREQUAL "openmp")
#  MESSAGE(STATUS "Using OpenMP linear algebra execution mode")
#  FIND_PACKAGE(OpenMP REQUIRED)
#  IF(OPENMP_FOUND)
#    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -DVIENNACL_WITH_OPENMP")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DVIENNACL_WITH_OPENMP")
#  ENDIF(OPENMP_FOUND)
#ENDIF()

INCLUDE_DIRECTORIES(".")

# ------------------------------------------------------------------------------
#
# INCLUDE EXTERNAL PROJECTS
#
# ------------------------------------------------------------------------------
IF(DOWNLOAD)
  MESSAGE(STATUS "Downloading ViennaStar during the build step ..")
ELSE(DOWNLOAD)
  MESSAGE(STATUS "Using local ViennaStar via VIENNA*PATH environment variables ..")
ENDIF(DOWNLOAD)

INCLUDE(cmake/GetViennaFVM.cmake)
MESSAGE(STATUS "ViennaFVM Include Path: " ${VIENNAFVM_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${VIENNAFVM_INCLUDE_DIRS})

INCLUDE(cmake/GetViennaGrid.cmake)
MESSAGE(STATUS "ViennaGrid Include Path: " ${VIENNAGRID_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${VIENNAGRID_INCLUDE_DIRS})

INCLUDE(cmake/GetViennaData.cmake)
MESSAGE(STATUS "ViennaData Include Path: " ${VIENNADATA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${VIENNADATA_INCLUDE_DIRS})

INCLUDE(cmake/GetViennaMath.cmake)
MESSAGE(STATUS "ViennaMath Include Path: " ${VIENNAMATH_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${VIENNAMATH_INCLUDE_DIRS})

INCLUDE(cmake/GetViennaCL.cmake)
MESSAGE(STATUS "ViennaCL Include Path: " ${VIENNACL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${VIENNACL_INCLUDE_DIRS})

INCLUDE(cmake/GetViennaMaterials.cmake)
MESSAGE(STATUS "ViennaMaterials Include Path: " ${VIENNAMATERIALS_INCLUDE_DIRS})
MESSAGE(STATUS "ViennaMaterials Libraries:    " ${VIENNAMATERIALS_LIBRARIES})
INCLUDE_DIRECTORIES(${VIENNAMATERIALS_INCLUDE_DIRS})
SET(LIBRARIES ${LIBRARIES} ${VIENNAMATERIALS_LIBRARIES})

# Currently, we only need ViennaMesh for the examples
IF(BUILD_EXAMPLES)
  INCLUDE(cmake/GetViennaMesh.cmake)
  MESSAGE(STATUS "ViennaMesh Include Path: " ${VIENNAMESH_INCLUDE_DIRS})
  MESSAGE(STATUS "ViennaMesh Libraries:    " ${VIENNAMESH_LIBRARIES})
  INCLUDE_DIRECTORIES(${VIENNAMESH_INCLUDE_DIRS})
  SET(LIBRARIES ${LIBRARIES} ${VIENNAMESH_LIBRARIES})
ENDIF(BUILD_EXAMPLES)

FIND_PACKAGE ( Threads REQUIRED ) # required by ViennaMesh
SET(LIBRARIES ${LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# following required by ViennaMaterials: TODO pimp the cmake module
FIND_PACKAGE(PythonLibs REQUIRED)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
SET(LIBRARIES ${LIBRARIES} ${PYTHON_LIBRARY})
ADD_DEFINITIONS(-DVIENNAMATERIALS_WITH_PYTHON)

FIND_PACKAGE(ViennaDeviceCollection)
IF(VIENNADEVICECOLLECTION_FOUND)
  CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/viennamini/device_collection.hpp.in
                 ${CMAKE_BINARY_DIR}/viennamini/device_collection.hpp)
  INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
ENDIF(VIENNADEVICECOLLECTION_FOUND)

IF(USE_UNITS)
  FIND_PACKAGE(UDUNITS REQUIRED)
  IF(UDUNITS_FOUND)
    INCLUDE_DIRECTORIES(${UDUNITS_INCLUDE_DIR})
    SET(LIBRARIES ${LIBRARIES} ${UDUNITS_LIBRARIES})
    ADD_DEFINITIONS(-DVIENNAMINI_WITH_UNITS)
    MESSAGE(STATUS "ViennaMini will be built with support for automatic unit conversions!")
  ELSE(UDUNITS_FOUND)
    MESSAGE(WARNING "ViennaMini will be built without support for automatic unit conversions!")
  ENDIF(UDUNITS_FOUND)
ELSE(USE_UNITS)
  MESSAGE(STATUS "ViennaMini will be built without support for automatic unit conversions!")
ENDIF(USE_UNITS)

#ADD_SUBDIRECTORY(external)
#SET(LIBRARIES ${LIBRARIES} pugixml)

# build the ViennaMini library
#AUX_SOURCE_DIRECTORY(src/ LIBSOURCES)
FILE(GLOB_RECURSE LIBSOURCES src/*.cpp)
ADD_LIBRARY(viennamini ${LIBSOURCES})
IF(NOT BUILD_SHARED_LIBS)
  SET_TARGET_PROPERTIES(viennamini PROPERTIES COMPILE_FLAGS "-fPIC")
ENDIF(NOT BUILD_SHARED_LIBS)
TARGET_LINK_LIBRARIES(viennamini ${LIBRARIES})
ADD_DEPENDENCIES(viennamini viennamaterials viennamesh)
SET(LIBRARIES ${LIBRARIES} viennamini)

# build the application
ADD_EXECUTABLE(viennamini-app     app/viennamini_app.cpp)
TARGET_LINK_LIBRARIES(viennamini-app ${LIBRARIES})

# build the examples
IF(BUILD_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
ENDIF(BUILD_EXAMPLES)

#IF(BUILD_TESTS)
#  ADD_SUBDIRECTORY(tests)
#ENDIF(BUILD_TESTS)


##Compatibility with Qt-Creator
file( GLOB_RECURSE QtCreatorCompatibility_SRC
    viennamini/*.hpp viennamini/*.h)
add_custom_target( QtCreatorCompatibility SOURCES ${QtCreatorCompatibility_SRC} )
