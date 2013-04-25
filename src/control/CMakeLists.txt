add_eeros_sources(Block.cpp AnSignal.cpp Block1i.cpp BlockOutput.cpp Block1o.cpp Input.cpp Output.cpp Step.cpp TimeDomain.cpp)

#link_directories (${EEROS_BINARY_DIR}/src/control)
#add_library(eeros_control SHARED )