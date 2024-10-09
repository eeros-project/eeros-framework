#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_

#include <eeros/control/ros2/RosPublisher.hpp>
#include <example_interfaces/msg/float64.hpp>

namespace eeros {
namespace control {

/**
 * This block allows to read a single input signal of type double and
 * publishes it as a ROS message of type std_msgs::msg::Float64.
 *
 * @since v1.0
 */
class RosPublisherDouble : public RosPublisher<example_interfaces::msg::Float64, double> {
  typedef example_interfaces::msg::Float64 TRosMsg;

 public:
  /**
   * Creates an instance of a publisher block which publishes a input signal
   * of type double as a ROS message of type std_msgs::msg::Float64.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisherDouble(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize = 1000)
      : RosPublisher<TRosMsg, double>(node, topic, queueSize) {}

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherDouble(const RosPublisherDouble& other) = delete;

  /**
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) { msg.data = in.getSignal().getValue(); }
};

}  // namespace control
}  // namespace eeros

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLE_HPP_ */
