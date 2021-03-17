set(targets "")

add_executable(sequencerTest10 SequencerTest10.cpp)
target_link_libraries(sequencerTest10 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest10)

add_executable(sequencerTest11 SequencerTest11.cpp)
target_link_libraries(sequencerTest11 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest11)

add_executable(sequencerTest12 SequencerTest12.cpp)
target_link_libraries(sequencerTest12 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest12)

add_executable(sequencerTest13 SequencerTest13.cpp)
target_link_libraries(sequencerTest13 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest13)

add_executable(sequencerTest14 SequencerTest14.cpp)
target_link_libraries(sequencerTest14 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest14)

add_executable(sequencerTest20 SequencerTest20.cpp)
target_link_libraries(sequencerTest20 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest20)

add_executable(sequencerTest21 SequencerTest21.cpp)
target_link_libraries(sequencerTest21 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest21)

add_executable(sequencerTest22 SequencerTest22.cpp)
target_link_libraries(sequencerTest22 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest22)

add_executable(sequencerTest23 SequencerTest23.cpp)
target_link_libraries(sequencerTest23 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest23)

add_executable(sequencerTest24 SequencerTest24.cpp)
target_link_libraries(sequencerTest24 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest24)

add_executable(sequencerTest25 SequencerTest25.cpp)
target_link_libraries(sequencerTest25 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest25)

add_executable(sequencerTest26 SequencerTest26.cpp)
target_link_libraries(sequencerTest26 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest26)

add_executable(sequencerTest30 SequencerTest30.cpp)
target_link_libraries(sequencerTest30 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest30)

add_executable(sequencerTest31 SequencerTest31.cpp)
target_link_libraries(sequencerTest31 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest31)

add_executable(sequencerTest32 SequencerTest32.cpp)
target_link_libraries(sequencerTest32 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest32)

add_executable(sequencerTest33 SequencerTest33.cpp)
target_link_libraries(sequencerTest33 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest33)

add_executable(sequencerTest34 SequencerTest34.cpp)
target_link_libraries(sequencerTest34 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest34)

add_executable(sequencerTest35 SequencerTest35.cpp)
target_link_libraries(sequencerTest35 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest35)

add_executable(sequencerTest36 SequencerTest36.cpp)
target_link_libraries(sequencerTest36 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest36)

add_executable(sequencerTest40 SequencerTest40.cpp)
target_link_libraries(sequencerTest40 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest40)

add_executable(sequencerTest41 SequencerTest41.cpp)
target_link_libraries(sequencerTest41 eeros ${EEROS_LIBS})
list(APPEND targets sequencerTest41)

if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/sequencer)
endif()

