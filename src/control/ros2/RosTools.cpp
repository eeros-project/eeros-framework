#include <eeros/control/ros2/RosTools.hpp>

namespace eeros {
namespace control {

void (*RosTools::customHandler)(int) = nullptr;
void (*RosTools::ros2Handler)(int) = nullptr;

}
}
