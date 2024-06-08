#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <std_msgs/msg/float64.h>

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type std_msgs::Float64::Type
 * and publishes it as a signal of type double.
 * 
 * @since v1.0
 *
 * @deprecated Since Foxy
 */
class RosSubscriberDouble : public RosSubscriber<std_msgs::msg::Float64, double> {
  typedef std_msgs::msg::Float64 TRosMsg;
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type std_msgs::Float64::Type under a given topic 
   * and outputs its values onto a signal of type double. 
   * If several messages are pending for a given topic
   * all the messages are consumed and the signal is set to the
   * newest.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberDouble(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic=false, const uint32_t queueSize=1000)
      : RosSubscriber<TRosMsg, double>(node, topic, syncWithTopic, queueSize) { }
    
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosSubscriberDouble(const RosSubscriberDouble& other) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   * 
   * @param msg - message content
   */
  virtual void parseMsg(const TRosMsg& msg) {
    auto time = eeros::System::getTimeNs();	// use system time for timestamp
    this->out.getSignal().setTimestamp( time );
    this->out.getSignal().setValue(static_cast<double>(msg.data) );
  }
};

}
}

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP
