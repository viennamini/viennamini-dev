

ADD_EXECUTABLE(nin2d     nin2d.cpp)
TARGET_LINK_LIBRARIES(nin2d ${LIBRARIES})

ADD_EXECUTABLE(mosfet     mosfet.cpp)
TARGET_LINK_LIBRARIES(mosfet ${LIBRARIES})

ADD_EXECUTABLE(trigate     trigate.cpp)
TARGET_LINK_LIBRARIES(trigate ${LIBRARIES})

#ADD_EXECUTABLE(device_generator     device_generator.cpp)
#TARGET_LINK_LIBRARIES(device_generator ${LIBRARIES} ${VIENNAMESH_TRIANGLE})



