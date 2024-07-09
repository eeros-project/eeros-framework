#include <eeros/control/ros2/EerosRosTools.hpp>

namespace eeros {
namespace control {
namespace rosTools {

void (*EerosRos2Tools::customHandler)(int) = nullptr;
void (*EerosRos2Tools::ros2Handler)(int) = nullptr;

} /* End Namespace: rosTools */
} /* End Namespace: control */
} /* End Namespace: eeros */