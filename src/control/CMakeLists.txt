include_directories(${EEROS_SOURCE_DIR}/includes)
link_directories (${EEROS_BINARY_DIR}/src/control)

add_library(eeros_control SHARED Block.cpp Block1i.cpp Block1o.cpp Input.cpp Output.cpp Signal.cpp Step.cpp System.cpp TextOutput.cpp)