#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros_msgs/msg/digital_signal.hpp>
#include <ostream>

namespace eeros::control {

/**
 * @brief Subscribes to a ROS2 @c DigitalSignal topic and exposes it as a scalar or vector output.
 *
 * Reads @c eeros_msgs::msg::DigitalSignal messages and writes their payload
 * to the block's output signal. The output timestamp is derived from the
 * message timestamp (converted from ROS to EEROS time).
 *
 * @tparam SigOutType  Output signal type — @c bool or a compound vector type
 *                     (e.g. @c Matrix<3,1,bool>). Default: @c bool
 *
 * @par Example
 * @code
 * RosSubscriberDigitalSignal<bool>           sub(node, "/sensor/enabled");
 * RosSubscriberDigitalSignal<Matrix<3,1,bool>> subVec(node, "/sensor/buttons");
 * @endcode
 *
 * @since v1.0
 */
template < typename SigOutType = bool >
class RosSubscriberDigitalSignal : public RosSubscriber<eeros_msgs::msg::DigitalSignal, 1, SigOutType> {

  using TRosMsg = eeros_msgs::msg::DigitalSignal;
  using Base = RosSubscriber<TRosMsg, 1, SigOutType>;

 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type eeros_msgs::msg::DigitalSignal under a given topic
   * and outputs its values onto a signal of type bool or vector of bool.
   * If several messages are pending for a given topic
   * all the messages are consumed and the signal is set to the
   * newest.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberDigitalSignal(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic = false,
                      const uint32_t queueSize = 1000)
      : Base(std::move(node), topic, syncWithTopic, queueSize) {}

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosSubscriberDigitalSignal(const RosSubscriberDigitalSignal&) = delete;
  RosSubscriberDigitalSignal& operator=(const RosSubscriberDigitalSignal&) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   *
   * @param msg - message content
   */
  void parseMsg(const TRosMsg& msg) override {
    if constexpr (std::is_integral_v<SigOutType>) {
      this->getOut().getSignal().setValue(msg.val[0]);
    } else {
      SigOutType val{};
      for (uint32_t i = 0; i < val.size(); ++i)
        val[i] = msg.val[i];
      this->getOut().getSignal().setValue(val);
    }
    this->getOut().getSignal().setTimestamp(RosTools::convertToEerosTime(msg.timestamp));
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, const RosSubscriberDigitalSignal<>& s) {
  os << "Block RosSubscriberDigitalSignal: '" << s.getName() << "'";
  return os;
}

}
