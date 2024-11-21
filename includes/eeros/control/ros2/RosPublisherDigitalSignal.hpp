#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros_msgs/msg/digital_signal.hpp>

namespace eeros {
namespace control {

/**
 * This block allows to read a single digital input signal of type bool or vector of bool and
 * publishes it as a ROS message of type eeros_msgs::msg::DigitalSignal.
 *
 * @tparam SigInType - type of the input signal
 *
 * @since v1.0
 */
template < typename SigInType = bool >
class RosPublisherDigitalSignal : public RosPublisher<eeros_msgs::msg::DigitalSignal, 1, SigInType> {
  typedef eeros_msgs::msg::DigitalSignal TRosMsg;

 public:
  /**
   * Creates an instance of a publisher block which publishes a input signal
   * of type bool or vector of bool as a ROS message of type std_msgs::msg::Float64.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisherDigitalSignal(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize = 1000)
      : RosPublisher<TRosMsg, 1, SigInType>(node, topic, queueSize) {}

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherDigitalSignal(const RosPublisherDigitalSignal& other) = delete;

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
   template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _set(TRosMsg& msg) {
     msg.val = {this->in.getSignal().getValue()};
   }
   template <typename S> typename std::enable_if<std::is_compound<S>::value>::type _set(TRosMsg& msg) {
     msg.val = this->in.getSignal().getValue().getColVector(0);
   }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, RosPublisherDigitalSignal<>& p) {
  os << "Block RosPublisherDigitalSignal: '" << p.getName();
  return os;
}

}
}
