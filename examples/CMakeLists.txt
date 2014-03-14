
IF(VIENNADEVICECOLLECTION_FOUND)
  ADD_EXECUTABLE(new_problem     new_problem.cpp)
  TARGET_LINK_LIBRARIES(new_problem ${LIBRARIES})

  ADD_EXECUTABLE(nin2d_poisson_dd_np     nin2d_poisson_dd_np.cpp)
  TARGET_LINK_LIBRARIES(nin2d_poisson_dd_np ${LIBRARIES})

  ADD_EXECUTABLE(mosfet2d_poisson_dd_np     mosfet2d_poisson_dd_np.cpp)
  TARGET_LINK_LIBRARIES(mosfet2d_poisson_dd_np ${LIBRARIES})

  ADD_EXECUTABLE(trigate3d_poisson_dd_np     trigate3d_poisson_dd_np.cpp)
  TARGET_LINK_LIBRARIES(trigate3d_poisson_dd_np ${LIBRARIES})

  ADD_EXECUTABLE(nin2d_laplace     nin2d_laplace.cpp)
  TARGET_LINK_LIBRARIES(nin2d_laplace ${LIBRARIES})

  ADD_EXECUTABLE(mosfet_planar_3d     mosfet_planar_3d.cpp)
  TARGET_LINK_LIBRARIES(mosfet_planar_3d ${LIBRARIES})
ENDIF(VIENNADEVICECOLLECTION_FOUND)

# For the following examples, we do not need the ViennaDeviceCollection
#
ADD_EXECUTABLE(template_capacitor1d     template_capacitor1d.cpp)
TARGET_LINK_LIBRARIES(template_capacitor1d ${LIBRARIES} ${VIENNAMESH_LIBRARIES})

ADD_EXECUTABLE(template_capacitor2d     template_capacitor2d.cpp)
TARGET_LINK_LIBRARIES(template_capacitor2d ${LIBRARIES} ${VIENNAMESH_LIBRARIES})

ADD_EXECUTABLE(template_capacitor3d     template_capacitor3d.cpp)
TARGET_LINK_LIBRARIES(template_capacitor3d ${LIBRARIES} ${VIENNAMESH_LIBRARIES})

ADD_EXECUTABLE(template_diode_np2d     template_diode_np2d.cpp)
TARGET_LINK_LIBRARIES(template_diode_np2d ${LIBRARIES} ${VIENNAMESH_LIBRARIES})

ADD_EXECUTABLE(diode_np2d_characteristics     diode_np2d_characteristics.cpp)
TARGET_LINK_LIBRARIES(diode_np2d_characteristics ${LIBRARIES} ${VIENNAMESH_LIBRARIES})
