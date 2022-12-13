#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <eeros/control/ros/RosPublisher.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Block.hpp>

using namespace eeros::safety;

namespace eeros {
namespace control {

/**
 * This block allows to publish the safety level of the safety SafetySystem
 * as a ROS message of type std_msgs::msg::UInt32::Type.
 * 
 * @since v1.0
 */
class RosPublisherSafetyLevel : public Block {
  typedef std_msgs::msg::UInt32::Type TRosMsg;
  
 public:
  /**
   * Creates an instance of a publisher block which publishes the safety level.
   * 
   * @param node - nThe ROS Node as a SharedPtr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  RosPublisherSafetyLevel(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000)
      : handle(node),
        topic(topic),
        log(logger::Logger::getLogger()) {
    if (rclcpp::ok()) {
      publisher = handle->create_publisher<TRosMsg>(topic, queueSize);
      log.trace() << "RosPublisherSafetyLevel, reading from topic: '" << topic << "' on node '" << node->get_name() << "' created.";
      running = true;
    }
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherSafetyLevel(const RosPublisherSafetyLevel& other) = delete;

  /**
   * Stores a reference to the safety system. Call this function after creating the 
   * control system and the safety system.
   * 
   * @param ss - safety system
   */
  void setSafetySystem(SafetySystem& ss) {
    safetySystem = &ss;
  }
  
  /**
   * This method will be executed whenever the block runs.
   */
  virtual void run() {
    if (running && safetySystem != nullptr) {
      SafetyLevel sl = safetySystem->getCurrentLevel();
      msg.data = sl.getLevelId();
      publisher->publish(msg);
    }
  }

  /**
   * This logger is used to put out information about the safety system
   */
  logger::Logger log;
  
  protected:
    rclcpp::Node::SharedPtr handle;
    typename rclcpp::Publisher<TRosMsg>::SharedPtr publisher;
    const std::string& topic;
    TRosMsg msg;
    SafetySystem* safetySystem;
    bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_ */
