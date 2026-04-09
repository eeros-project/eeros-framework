#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>

#include <geometry_msgs/msg/twist.hpp>


namespace eeros::control {

/**
 * This block subscribes to a 'geometry_msgs::msg::Twist' message and outputs its content as signals.
 * The block has 2 output vectors of type 'Vector3' with
 * - index 0: twist linear
 * - index 1: twist angular
 */
class RosSubscriberTwist : public RosSubscriber<geometry_msgs::msg::Twist, 2, math::Vector3> {

  using TRosMsg = geometry_msgs::msg::Twist;
  using Base = RosSubscriber<TRosMsg, 2, eeros::math::Vector3>;

 public:

  /**
   * Creates an instance of a subscriber block which reads a twist message
   * of type geometry_msgs::msg::Twist.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberTwist(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic = false,
                      const uint32_t queueSize=1000)
      : Base(std::move(node), topic, syncWithTopic, queueSize) { }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosSubscriberTwist(const RosSubscriberTwist&) = delete;
  RosSubscriberTwist& operator=(const RosSubscriberTwist&) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   *
   * @param msg - message content
   */
  void parseMsg(const TRosMsg& msg) override {
    const auto now = eeros::System::getTimeNs();
    out[0].getSignal().setValue({msg.linear.x, msg.linear.y, msg.linear.z});
    out[0].getSignal().setTimestamp(now);
    out[1].getSignal().setValue({msg.angular.x, msg.angular.y, msg.angular.z});
    out[1].getSignal().setTimestamp(now);
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, const RosSubscriberTwist& s) {
  os << "Block RosSubscriberTwist: '" << s.getName() << "'";
  return os;
}

}
