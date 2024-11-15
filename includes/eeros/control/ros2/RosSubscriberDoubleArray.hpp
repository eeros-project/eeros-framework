#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <example_interfaces/msg/float32_multi_array.hpp>

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type std_msgs::Float64MultiArray::Type
 * and publishes it as a signal of type Matrix<N,1,double>.
 *
 * @tparam SigOutType - type of the output signal
 * @since v1.0
 */
template <typename SigOutType>
class RosSubscriberDoubleArray : public RosSubscriber<example_interfaces::msg::Float64MultiArray, 1, SigOutType> {
  typedef example_interfaces::msg::Float64MultiArray TRosMsg;

 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type std_msgs::Float64MultiArray::Type under a given topic
   * and outputs its values onto a signal of type Matrix<N,1,double>.
   * If several messages are pending for a given topic
   * all the messages are consumed and the signal is set to the
   * newest.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberDoubleArray(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic = false,
                           const uint32_t queueSize = 1000)
      : RosSubscriber<TRosMsg, 1, SigOutType>(node, topic, syncWithTopic, queueSize) {}

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosSubscriberDoubleArray(const RosSubscriberDoubleArray& other) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   *
   * @param msg - message content
   */
  virtual void parseMsg(const TRosMsg& msg) override {
    auto time = eeros::System::getTimeNs();  // use system time for timestamp
    this->out.getSignal().setTimestamp(time);
    std::vector<double> valTmp(msg.data.begin(), msg.data.end());
    val.setCol(0, valTmp);
    this->out.getSignal().setValue(val);
  }

 protected:
  SigOutType val;
};

/********** Print functions **********/
template <typename T>
std::ostream& operator<<(std::ostream& os, RosSubscriberDoubleArray<T>& s) {
  os << "Block RosSubscriberDoubleArray: '" << s.getName();
  return os;
}

}
}

