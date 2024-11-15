#pragma once

#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <eeros/logger/Logger.hpp>

#define NS_PER_SEC 1000000000

using namespace eeros::logger;

namespace eeros {
namespace control {

/**
 * This is a fully static class with ROS2 helper functions.
 * You have to use initRos() first.
 */
class RosTools {
 public:

  /* Delete ctors */
  RosTools() = delete;
  RosTools(const RosTools&) = delete;
  RosTools(const RosTools&&) = delete;
  RosTools& operator=(const RosTools&) = delete;

  /**
   * Initialize rclcpp, read the enviroment variable "ROS_DOMAIN_ID"
   * to set the domain id where ros2 has to be run.
   * This registers a function that will be called when the program ends to shut down rclcpp.
   */
  static void initRos(int argc, char** argv) {
    auto log = Logger::getLogger();
    if(!rclcpp::ok()) {
      rclcpp::InitOptions options;
      const char* domain_id_env = std::getenv("ROS_DOMAIN_ID");
      if (domain_id_env) {
        try {
          int domain_id = std::stoi(domain_id_env);
          options.set_domain_id(domain_id);
          log.trace() << "Set DOMAIN_ID to " << domain_id;
        } catch (const std::exception& e) {
          log.error() << "Error: invalid enviroment variable: ROS_DOMAIN_ID: " << domain_id_env;
          log.trace() << " --> Using default: 0";
          options.set_domain_id(0);
        }
      } else {
        options.set_domain_id(0); /* Default */
        log.trace() << "ROS_DOMAIN_ID not set. Using default: 0";
      }
      std::atexit(cleanup); /* add function to call rclcpp::shutdown at the end of the program */
      rclcpp::init(argc, argv, options);
    } else {
      log.warn() << "Tried to initialize Ros2(rclcpp) multiple times";
    }
  }

  /**
   * This function registers your own SIGINT handler along the ROS2 SIGINT handler.
   * Call this immediately after the initRos2 function.
   */
  static void registerCustomSigIntHandler(void (customHandler)(int)) {
    if (rclcpp::ok()) {
      if(RosTools::customHandler != nullptr) {
        Logger::getLogger().warn() << "Custom SIGINT handler already registerd along ROS2 SIGINT handler";
        return;
      }
      RosTools::customHandler = customHandler;

      struct sigaction sa;
      memset(&sa, 0, sizeof(sa));
      sigaction(SIGINT, NULL, &sa);
      ros2Handler = sa.sa_handler;

      memset(&sa, 0, sizeof(sa));
      sa.sa_handler = aggregateHandler;
      sigemptyset(&sa.sa_mask);
      sigaction(SIGINT, &sa, NULL);
    }
  }

  /**
   * Creates and initializes a ROS node. Checks if ROS master is up and running.
   * Returns false if no ROS master can be contacted.
   *
   * @param name - name of the node
   * @param nameSpace - namespace of the node
   * @param letNodeSpin - todo

   * @param return node
   */
  static rclcpp::Node::SharedPtr initNode(std::string name, std::string nameSpace = "", bool letNodeSpin = false) {
    if (rclcpp::ok()) {
      rclcpp::NodeOptions opt;
      rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(name, nameSpace);
      if(letNodeSpin) {
        std::thread([node]() {
          spin(node); /* Terminates when the node is destroyed or when rclcpp::shutdown is called. */
        }).detach();
      }
      return node;
    }
    return nullptr;
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
   * Creates an EEROS timestamp (nanoseconds) based on a ROS message header timestamp struct.
   *
   * @param time - the ROS-Message timestamp to convert
   * @return EEROS timestampNs
   */
  static uint64_t toNanoSec(const builtin_interfaces::msg::Time& time) {
    return static_cast<uint64_t>(time.sec) * NS_PER_SEC + static_cast<uint64_t>(time.nanosec);
  }

 protected:
  static void (*customHandler)(int);
  static void (*ros2Handler)(int);

  static void aggregateHandler(int signum) {
    if (ros2Handler != nullptr) {
      ros2Handler(signum);
      eeros::logger::Logger::getLogger().trace() << "Called Ros2 SIGINT Handler";
    }
    if (customHandler != nullptr) {
      customHandler(signum);
      eeros::logger::Logger::getLogger().trace() << "Called custom SIGINT Handler";
    }
  }

  // called when shutting down
  static void cleanup(){
    if(rclcpp::ok()) {
      logger::Logger::getLogger().trace() << "rclcpp shutdown";
      rclcpp::shutdown();
    }
  }
};

}
}
