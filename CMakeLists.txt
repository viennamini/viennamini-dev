PROJECT(ViennaMini)

cmake_minimum_required(VERSION 2.6)

#if you don't want the full compiler output, remove the following line
SET(CMAKE_VERBOSE_MAKEFILE ON)

#specify custom boost path if needed:
#SET(BOOSTPATH "/opt/boost")

# alternate boost path for macs
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   INCLUDE_DIRECTORIES("/opt/local/include")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


#specify include and source directory
INCLUDE_DIRECTORIES(".")
INCLUDE_DIRECTORIES(${BOOSTPATH})
INCLUDE_DIRECTORIES("external/ViennaCL")
INCLUDE_DIRECTORIES("external/ViennaData")
INCLUDE_DIRECTORIES("external/ViennaFVM")
INCLUDE_DIRECTORIES("external/ViennaGrid")
INCLUDE_DIRECTORIES("external/ViennaMaterials")
INCLUDE_DIRECTORIES("external/ViennaMath")
INCLUDE_DIRECTORIES("external/ViennaUtils")

#set default build type 
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

#set compiler flags
SET(CMAKE_CXX_FLAGS "-Wall -pedantic")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")

#list all source files here
ADD_EXECUTABLE(mosfet     examples/tutorial/mosfet.cpp)
TARGET_LINK_LIBRARIES(mosfet ${LIBRARIES})

ADD_EXECUTABLE(trigate     examples/tutorial/trigate.cpp)
TARGET_LINK_LIBRARIES(trigate ${LIBRARIES})

ADD_EXECUTABLE(nin2d     examples/tutorial/nin2d.cpp)
TARGET_LINK_LIBRARIES(nin2d ${LIBRARIES})


##Compatibility with Qt-Creator
file( GLOB_RECURSE QtCreatorCompatibility_SRC
    viennamini/*.hpp viennamini/*.h)
add_custom_target( QtCreatorCompatibility SOURCES ${QtCreatorCompatibility_SRC} )