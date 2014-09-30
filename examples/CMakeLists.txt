
IF(VIENNADEVICECOLLECTION_FOUND)
  ADD_EXECUTABLE(2d_capacitor     2d_capacitor.cpp)
  TARGET_LINK_LIBRARIES(2d_capacitor ${LIBRARIES})

  ADD_EXECUTABLE(2d_diode_nin     2d_diode_nin.cpp)
  TARGET_LINK_LIBRARIES(2d_diode_nin ${LIBRARIES})

  ADD_EXECUTABLE(2d_mos     2d_mos.cpp)
  TARGET_LINK_LIBRARIES(2d_mos ${LIBRARIES})

  ADD_EXECUTABLE(2d_mosfet     2d_mosfet.cpp)
  TARGET_LINK_LIBRARIES(2d_mosfet ${LIBRARIES})

  ADD_EXECUTABLE(3d_mosfet_trigate_1     3d_mosfet_trigate_1.cpp)
  TARGET_LINK_LIBRARIES(3d_mosfet_trigate_1 ${LIBRARIES})

  ADD_EXECUTABLE(3d_mosfet_trigate_2     3d_mosfet_trigate_2.cpp)
  TARGET_LINK_LIBRARIES(3d_mosfet_trigate_2 ${LIBRARIES})

  ADD_EXECUTABLE(3d_mosfet     3d_mosfet.cpp)
  TARGET_LINK_LIBRARIES(3d_mosfet ${LIBRARIES})

  ADD_EXECUTABLE(3d_mosfet_utb_soi     3d_mosfet_utb_soi.cpp)
  TARGET_LINK_LIBRARIES(3d_mosfet_utb_soi ${LIBRARIES})


  ADD_EXECUTABLE(Pi_Gate     Pi_Gate.cpp)
  TARGET_LINK_LIBRARIES(Pi_Gate ${LIBRARIES})
  
  ADD_EXECUTABLE(Omega_Gate     Omega_Gate.cpp)
  TARGET_LINK_LIBRARIES(Omega_Gate ${LIBRARIES})

  ADD_EXECUTABLE(Tri_Gate3     Tri_Gate3.cpp)
  TARGET_LINK_LIBRARIES(Tri_Gate3 ${LIBRARIES})

  ADD_EXECUTABLE(Double_Gate_FinFET2     Double_Gate_FinFET2.cpp)
  TARGET_LINK_LIBRARIES(Double_Gate_FinFET2 ${LIBRARIES})

  ADD_EXECUTABLE(Four_Gate     Four_Gate.cpp)
  TARGET_LINK_LIBRARIES(Four_Gate ${LIBRARIES})

  ADD_EXECUTABLE(Tri_Gate2     Tri_Gate2.cpp)
  TARGET_LINK_LIBRARIES(Tri_Gate2 ${LIBRARIES})

  ADD_EXECUTABLE(Double_Gate_MOSFET3     Double_Gate_MOSFET3.cpp)
  TARGET_LINK_LIBRARIES(Double_Gate_MOSFET3 ${LIBRARIES})

  ADD_EXECUTABLE(Double_Gate_MOSFET     Double_Gate_MOSFET.cpp)
  TARGET_LINK_LIBRARIES(Double_Gate_MOSFET ${LIBRARIES})

  ADD_EXECUTABLE(MOSFET     MOSFET.cpp)
  TARGET_LINK_LIBRARIES(MOSFET ${LIBRARIES})

  ADD_EXECUTABLE(Single_Gate     Single_Gate.cpp)
  TARGET_LINK_LIBRARIES(Single_Gate ${LIBRARIES})

ENDIF(VIENNADEVICECOLLECTION_FOUND)


