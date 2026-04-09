#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>
#include <ostream>
#include <concepts>

namespace eeros::control {

/**
 * This block allows to read a single analog input signal of type double or vector of double and
 * publishes it as a ROS message of type eeros_msgs::msg::AnalogSignal.
 *
 * @tparam SigInType - type of the input signal
 *
 * @since v1.0
 */
template < typename SigInType = double >
class RosPublisherAnalogSignal : public RosPublisher<eeros_msgs::msg::AnalogSignal, 1, SigInType> {

  using TRosMsg = eeros_msgs::msg::AnalogSignal;
  using Base = RosPublisher<TRosMsg, 1, SigInType>;

 public:
  /**
   * Creates an instance of a publisher block which publishes a input signal
   * of type double or vector of double as a ROS message of type eeros_msgs::msg::AnalogSignal.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisherAnalogSignal(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize = 1000)
      : Base(std::move(node), topic, queueSize) {}

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosPublisherAnalogSignal(const RosPublisherAnalogSignal& other) = delete;
  RosPublisherAnalogSignal& operator=(const RosPublisherAnalogSignal&) = delete;

  /**
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  void setRosMsg(TRosMsg& msg) override {
    if constexpr (std::is_floating_point_v<SigInType>)
      msg.val = {this->getIn().getSignal().getValue()};
    else
      msg.val = this->getIn().getSignal().getValue().getColVector(0);
    msg.timestamp = RosTools::convertToRosTime(this->getIn().getSignal().getTimestamp());
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, const RosPublisherAnalogSignal<>& p) {
  os << "Block RosPublisherAnalogSignal: '" << p.getName() << "'";
  return os;
}

}
