#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_

#include <eeros/control/ros2/RosPublisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <eeros/safety/SafetySystem.hpp>

using namespace eeros::safety;

namespace eeros {
namespace control {

/**
 * This block allows to read the safety level of the safety system and
 * publish it as a ROS message of type std_msgs::msg::UInt32.
 * 
 * @since v1.0
 */
class RosPublisherSafetyLevelName : public RosPublisher<std_msgs::msg::String, double> {
  typedef std_msgs::msg::String TRosMsg;
  
 public:
  /**
   * Creates an instance of a publisher block which publishes the safety level.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  RosPublisherSafetyLevelName(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000)
      : RosPublisher<TRosMsg, double>(node, topic, queueSize) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherSafetyLevelName(const RosPublisherSafetyLevelName& other) = delete;

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
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) {
    if (safetySystem != nullptr) {
      SafetyLevel sl = safetySystem->getCurrentLevel();
      msg.data = sl.getDescription();
    }
  }
  
 private:
  SafetySystem* safetySystem;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_ */
