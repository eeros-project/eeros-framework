set(targets "")

# if (sbgecomlib_FOUND)
    add_executable(sbg_test SBGEllipseATest.cpp)
    target_link_libraries(sbg_test eeros ${EEROS_LIBS} sbgECom)
    list(APPEND targets sbg_test)

    if(INSTALL_EXAMPLES)
    install(TARGETS ${targets} RUNTIME DESTINATION examples/block/sensors)
    endif()
# endif()
