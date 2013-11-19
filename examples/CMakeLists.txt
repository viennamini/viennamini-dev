

ADD_EXECUTABLE(nin2d_poisson_dd_np     nin2d_poisson_dd_np.cpp)
TARGET_LINK_LIBRARIES(nin2d_poisson_dd_np ${LIBRARIES})

ADD_EXECUTABLE(mosfet2d_poisson_dd_np     mosfet2d_poisson_dd_np.cpp)
TARGET_LINK_LIBRARIES(mosfet2d_poisson_dd_np ${LIBRARIES})

ADD_EXECUTABLE(trigate3d_poisson_dd_np     trigate3d_poisson_dd_np.cpp)
TARGET_LINK_LIBRARIES(trigate3d_poisson_dd_np ${LIBRARIES})

ADD_EXECUTABLE(nin2d_laplace     nin2d_laplace.cpp)
TARGET_LINK_LIBRARIES(nin2d_laplace ${LIBRARIES})



#ADD_EXECUTABLE(device_generator     device_generator.cpp)
#TARGET_LINK_LIBRARIES(device_generator ${LIBRARIES} ${VIENNAMESH_TRIANGLE})



