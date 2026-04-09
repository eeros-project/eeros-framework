#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>
#include <ostream>

namespace eeros::control {

/**
 * @brief Subscribes to a ROS2 @c AnalogSignal topic and exposes it as a scalar or vector output.
 *
 * Reads @c eeros_msgs::msg::AnalogSignal messages and writes their payload
 * to the block's output signal. The output timestamp is derived from the
 * message timestamp (converted from ROS to EEROS time).
 *
 * @tparam SigOutType  Output signal type — @c double or a compound vector type
 *                     (e.g. @c Vector3). Default: @c double
 *
 * @par Example
 * @code
 * RosSubscriberAnalogSignal<double>  sub(node, "/sensor/value");
 * RosSubscriberAnalogSignal<Vector3> subVec(node, "/sensor/vector");
 * @endcode
 *
 * @since v1.0
 */
template < typename SigOutType = double >
class RosSubscriberAnalogSignal : public RosSubscriber<eeros_msgs::msg::AnalogSignal, 1, SigOutType> {
  
  using TRosMsg = eeros_msgs::msg::AnalogSignal;
  using Base = RosSubscriber<TRosMsg, 1, SigOutType>;

 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type eeros_msgs::msg::AnalogSignal under a given topic
   * and outputs its values onto a signal of type double or vector of double.
   * If several messages are pending for a given topic
   * all the messages are consumed and the signal is set to the
   * newest.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberAnalogSignal(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic = false,
                      const uint32_t queueSize = 1000)
      : Base(node, topic, syncWithTopic, queueSize) {}

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosSubscriberAnalogSignal(const RosSubscriberAnalogSignal&) = delete;
  RosSubscriberAnalogSignal& operator=(const RosSubscriberAnalogSignal&) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   *
   * @param msg - message content
   */
  void parseMsg(const TRosMsg& msg) override {
    if constexpr (std::is_floating_point_v<SigOutType>) {
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
std::ostream& operator<<(std::ostream& os, const RosSubscriberAnalogSignal<>& s) {
  os << "Block RosSubscriberAnalogSignal: '" << s.getName() << "'";
  return os;
}

}
