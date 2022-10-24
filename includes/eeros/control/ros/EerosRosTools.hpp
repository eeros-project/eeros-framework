#ifndef EEROS_ROS_TOOLS_HPP
#define EEROS_ROS_TOOLS_HPP

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define NS_PER_SEC 1000000000

namespace eeros {
namespace control {
namespace rosTools {
  
/**
 * Helper functions for using ROS.
 * 
 * @since v1.0
 */
  

/**
 * Converts a EEROS timestamp, which is in ns to ROS time in s.
 * 
 * @param timestampNs - EEROS timestamp
 * @return ROS time
 */
static builtin_interfaces::msg::Time convertToRosTime(uint64_t timestampNs) __attribute__((unused));
static builtin_interfaces::msg::Time convertToRosTime(uint64_t timestampNs) {
  builtin_interfaces::msg::Time t;
  t.set__sec(static_cast<double>(timestampNs) / NS_PER_SEC);
  t.set__nanosec(timestampNs % static_cast<uint64_t>(1e9));
  return t;
}


/**
 * Creates and initializes a ROS node. Checks if ROS master is up and running.
 * Returns false if no ROS master can be contacted.
 * 
 * @param name - name of the ROS node
 * @param return true, if ROS master can be found
 */
static bool initNode(std::string name) __attribute__((unused));
static bool initNode(std::string name) {
  char* args[] = {NULL};
  int argc = sizeof(args)/sizeof(args[0]) - 1;
  // init ROS node and name it
  rclcpp::init(argc, args);
  auto n = rclcpp::Node::make_shared(name);
  return rclcpp::ok();
}

}
}
}

#endif /* EEROS_ROS_TOOLS_HPP */
