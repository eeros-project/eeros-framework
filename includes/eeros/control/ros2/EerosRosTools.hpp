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
 * Creates an EEROS timestamp (nanoseconds) based on a ROS-Message-Header timestamp struct.
 *
 * @param time - the ROS-Message timestamp to convert
 * @return EEROS timestampNs
 */
static uint64_t toNanoSec(const builtin_interfaces::msg::Time& time) __attribute__((unused));
static uint64_t toNanoSec(const builtin_interfaces::msg::Time& time) {
  return static_cast<uint64_t>(time.sec) * NS_PER_SEC + static_cast<uint64_t>(time.nanosec);
}


/**
 * Creates and initializes a ROS node.
 * Returns a shared pointer to the ROS Node.
 *
 * @param name - name of the ROS node
 * @param return The ROS Node as a shared pointer
 */
static rclcpp::Node::SharedPtr initNode(std::string name) __attribute__((unused));
static rclcpp::Node::SharedPtr initNode(std::string name) {
  rclcpp::init(0, NULL);
  return rclcpp::Node::make_shared(name);
}


}
}
}

#endif /* EEROS_ROS_TOOLS_HPP */
