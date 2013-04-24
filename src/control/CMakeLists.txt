include_directories(${EEROS_SOURCE_DIR}/includes)
link_directories (${EEROS_BINARY_DIR}/src/control)

add_library(eeros_control SHARED TimeDomain.cpp Block.cpp Block1i.cpp BlockOutput.cpp Block1o.cpp Input.cpp Output.cpp AnSignal.cpp Step.cpp System.cpp)