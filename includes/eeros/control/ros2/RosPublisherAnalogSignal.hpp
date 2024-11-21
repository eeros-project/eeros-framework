#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>

namespace eeros {
namespace control {

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
  typedef eeros_msgs::msg::AnalogSignal TRosMsg;

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
      : RosPublisher<TRosMsg, 1, SigInType>(node, topic, queueSize) {}

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherAnalogSignal(const RosPublisherAnalogSignal& other) = delete;

  /**
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) {
    _set<SigInType>(msg);
    msg.timestamp = RosTools::convertToRosTime(this->in.getSignal().getTimestamp());
  }

 private:
  template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _set(TRosMsg& msg) {
    msg.val = {this->in.getSignal().getValue()};
  }
  template <typename S> typename std::enable_if<std::is_compound<S>::value>::type _set(TRosMsg& msg) {
    msg.val = this->in.getSignal().getValue().getColVector(0);
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, RosPublisherAnalogSignal<>& p) {
  os << "Block RosPublisherAnalogSignal: '" << p.getName();
  return os;
}

}
}
