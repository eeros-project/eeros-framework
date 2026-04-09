#pragma once

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <eeros/logger/Logger.hpp>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace eeros::control {

/**
 * @brief Fully static utility class providing ROS2 helper functions.
 *
 * Call @ref initRos() once at program startup before using any other ROS2
 * functionality. The class is non-instantiable and non-copyable.
 *
 * @par Typical usage
 * @code
 * RosTools::initRos(argc, argv);
 * auto node = RosTools::initNode("my_node");
 * RosTools::registerCustomSigIntHandler(myShutdownHandler);
 * @endcode
 */
class RosTools {
 public:

  /* Delete ctors */
  RosTools() = delete;
  RosTools(const RosTools&) = delete;
  RosTools(RosTools&&) = delete;
  RosTools& operator=(const RosTools&) = delete;
  RosTools& operator=(RosTools&&) = delete;

  /**
   * @brief Initialises @c rclcpp and selects the ROS2 domain.
   *
   * Reads the @c ROS_DOMAIN_ID environment variable to determine the domain.
   * Falls back to domain 0 if the variable is absent or invalid.
   * Registers @c rclcpp::shutdown() via @c std::atexit() so ROS is cleanly
   * shut down when the program exits.
   *
   * Calling this function more than once is safe — subsequent calls are ignored
   * with a warning.
   *
   * @param argc  Argument count forwarded from @c main()
   * @param argv  Argument vector forwarded from @c main()
   */
  static void initRos(int argc, char** argv) {
    auto log = eeros::logger::Logger::getLogger();
    if(rclcpp::ok()) {
      log.warn() << "Tried to initialize Ros2(rclcpp) multiple times";
      return;
    }
 
    rclcpp::InitOptions options;
    if (const char* domain_id_env = std::getenv("ROS_DOMAIN_ID")) {
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
  }

  /**
   * @brief Registers a custom SIGINT handler alongside the ROS2 SIGINT handler.
   *
   * Both handlers are called in order: ROS2 first, then the custom handler.
   * Must be called after @ref initRos(). Registering more than one custom
   * handler is not supported and will produce a warning.
   *
   * @param handler  Signal handler function with signature @c void(int)
   */
  static void registerCustomSigIntHandler(void (*handler)(int)) {
    if (!rclcpp::ok()) return;
    auto log = eeros::logger::Logger::getLogger();
    if(customHandler != nullptr) {
      log.warn() << "Custom SIGINT handler already registerd along ROS2 SIGINT handler";
      return;
    }
    customHandler = handler;

    struct sigaction sa{};
    sigaction(SIGINT, nullptr, &sa);
    ros2Handler = sa.sa_handler;

    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = aggregateHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
  }

  /**
   * @brief Creates a ROS2 node, optionally spinning it on a background thread.
   *
   * Returns @c nullptr if ROS is not running.
   *
   * @param name          Node name
   * @param nameSpace     Node namespace (default: empty)
   * @param letNodeSpin   If @c true, spins the node on a detached thread
   * @return              Shared pointer to the node, or @c nullptr
   */
  static rclcpp::Node::SharedPtr initNode(const std::string& name, const std::string& nameSpace = "", bool letNodeSpin = false) {
    if (!rclcpp::ok()) return nullptr;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(name, nameSpace);
    if(letNodeSpin) {
      std::thread([node]() {
        rclcpp::spin(node); /* Terminates when the node is destroyed or when rclcpp::shutdown is called. */
        }).detach();
      }
      return node;
  }

  /**
   * @brief Converts an EEROS timestamp (nanoseconds) to a ROS2 @c Time message.
   *
   * @param timestampNs  EEROS timestamp in nanoseconds
   * @return             Equivalent @c builtin_interfaces::msg::Time
   */
  static builtin_interfaces::msg::Time convertToRosTime(uint64_t timestampNs) {
    builtin_interfaces::msg::Time t;
    t.set__sec(static_cast<double>(timestampNs) / NS_PER_SEC);
    t.set__nanosec(timestampNs % static_cast<uint64_t>(1e9));
    return t;
  }

  /**
   * @brief Converts a ROS2 @c Time message to an EEROS timestamp (nanoseconds).
   *
   * @param timestamp  ROS2 timestamp
   * @return           Equivalent EEROS timestamp in nanoseconds
   */
  static uint64_t convertToEerosTime(const builtin_interfaces::msg::Time& timestamp) {
    return static_cast<uint64_t>(timestamp.sec) * NS_PER_SEC + static_cast<uint64_t>(timestamp.nanosec);
  }

  /**
   * @brief Shuts down @c rclcpp if it is still running.
   *
   * Registered via @c std::atexit() by @ref initRos() — not normally called directly.
   */
  static void cleanup(){
    if(rclcpp::ok()) {
      logger::Logger::getLogger().trace() << "rclcpp shutdown";
      rclcpp::shutdown();
    }
  }

 private:
  static constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
  static void (*customHandler)(int);
  static void (*ros2Handler)(int);

  static void aggregateHandler(int signum) {
    if (ros2Handler) {
      ros2Handler(signum);
      eeros::logger::Logger::getLogger().trace() << "Called Ros2 SIGINT Handler";
    }
    if (customHandler) {
      customHandler(signum);
      eeros::logger::Logger::getLogger().trace() << "Called custom SIGINT Handler";
    }
  }

};

}
