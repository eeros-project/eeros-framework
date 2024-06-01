#ifdef USE_ROS2 /* header is only for use with ros2 */

#ifndef EEROS_ROS_TOOLS_HPP
#define EEROS_ROS_TOOLS_HPP

#include <rclcpp/rclcpp.hpp>
#include <eeros/logger/Logger.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define NS_PER_SEC 1000000000

namespace eeros {
namespace control {
namespace rosTools {
  
/**
 * Helper functions for using ROS.
 * @since v1.0
 * @deprecated
 */
  

/**
 * @deprecated Use EerosRos2Tools::toNanoSec(...)
 * 
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
 * @deprecated Use EerosRos2Tools::toNanoSec(...)
 * 
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
 * @deprecated Use EerosRos2Tools::initNode(...)
 * 
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

/**
 * A fully static Class with Ros2 helpers
 * 
 * 
*/
class EerosRos2Tools {
  public:
  /* Delete this because this class is static only */
  EerosRos2Tools() = delete;
  EerosRos2Tools(const EerosRos2Tools&) = delete;
  EerosRos2Tools(const EerosRos2Tools&&) = delete;
  EerosRos2Tools& operator=(const EerosRos2Tools&) = delete;

  static void initRos2(int argc, char** argv){
    if(!rclcpp::ok) {
      rclcpp::InitOptions options;
      const char* domain_id_env = std::getenv("ROS_DOMAIN_ID");
      if (domain_id_env) {
        try {
          int domain_id = std::stoi(domain_id_env);
          options.set_domain_id(domain_id);
          logger::Logger::getLogger().trace() << "Set DOMAIN_ID to " << domain_id;
        } catch (const std::exception& e) {
          logger::Logger::getLogger().error() << "Error: invalid enviroment variable: ROS_DOMAIN_ID: " << domain_id_env;
          logger::Logger::getLogger().trace() << " --> Using default: 0";
          options.set_domain_id(0);
        }
      } else {
        options.set_domain_id(0);
        logger::Logger::getLogger().trace() << "ROS_DOMAIN_ID not set. Using default: 0";
      }
      std::atexit(cleanup); /* add function to call rclcpp::destroy at the end of the program */
      rclcpp::init(argc,argv, options);
    }
  }

  /**
   * Creates and initializes a ROS node.
   * Returns a shared pointer to the ROS Node.
   *
   * @param name - name of the ROS node
   * @param letNodeSpin - if set to true it wil start a seperate thread to spin the node (needed for services, asynchrounus communication, ...)
   * @return The ROS Node as a shared pointer
   */
  static rclcpp::Node::SharedPtr initNode(std::string nodeName, bool letNodeSpin = false) {
    EerosRos2Tools::initRos2(0, NULL);
    auto node = rclcpp::Node::make_shared(nodeName);
    if(letNodeSpin) {
      std::thread([node]() {
        rclcpp::spin(node); /* Terminates when the node is destroyed or when rclcpp::shutdown is called. */
      }).detach();
    }
    return node;
  }

  /**
   * Converts a EEROS timestamp, which is in ns to ROS time in s.
   *
   * @param timestampNs - EEROS timestamp
   * @return ROS time
   */
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
  static uint64_t toNanoSec(const builtin_interfaces::msg::Time& time) {
    return static_cast<uint64_t>(time.sec) * NS_PER_SEC + static_cast<uint64_t>(time.nanosec);
  }

  private:
  static void cleanup(){
    if(rclcpp::ok) {
      rclcpp::shutdown();
    }
  }

};

} /* Namespace rosTools */
} /* Namespace control */
} /* Namespace eeros */

#endif /* End EEROS_ROS_TOOLS_HPP*/

#endif /* End USE_ROS2 */
