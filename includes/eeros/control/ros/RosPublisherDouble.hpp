#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_

#include <eeros/core/System.hpp>
#include <eeros/control/ros/RosPublisher.hpp>
#include <std_msgs/msg/float64.hpp>

namespace eeros {
namespace control {

/**
 * This block allows to publish a single input signal of type double and
 * publishes it as a ROS message type std_msgs::msg::Float64::Type.
 * 
 * @since v1.0
 */
class RosPublisherDouble : public RosPublisher<std_msgs::msg::Float64::Type, double> {
  typedef std_msgs::msg::Float64::Type TRosMsg;
 
 public:
  /**
   * Creates an instance of a publisher block which publishes a input signal 
   * of type double as a ROS message of type std_msgs::msg::Float64::Type.
   * 
   * @param node_name - name of this node
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  RosPublisherDouble (const std::string& node_name, const std::string& topic, const uint32_t queueSize=1000)
      : RosPublisher<TRosMsg, double>(node_name, topic, queueSize) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherDouble(const RosPublisherDouble& other) = delete;

  /**
   * Sets the message to be published by this block.
   * 
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) {
    msg.data = in.getSignal().getValue();
  }
  
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_ */
