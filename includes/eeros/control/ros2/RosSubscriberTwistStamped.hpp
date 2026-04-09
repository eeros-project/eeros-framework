#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/math/Matrix.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>


namespace eeros::control {

/**
 * This block subscribes to a 'geometry_msgs::msg::TwistStamped' message and outputs its content as a signal.
 * The block has an output vector of type 'Matrix<6>' with
 */
class RosSubscriberTwistStamped : public RosSubscriber<geometry_msgs::msg::TwistStamped, 1, eeros::math::Matrix<6>> {

  using TRosMsg = geometry_msgs::msg::TwistStamped;
  using Base = RosSubscriber<TRosMsg,1,eeros::math::Matrix<6>>;

 public:

  /**
   * Creates an instance of a subscriber block which reads a twist message
   * of type geometry_msgs::msg::TwistStamped.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriberTwistStamped(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic = false,
                      const uint32_t queueSize=10)
      : Base(std::move(node), topic, syncWithTopic, queueSize) { }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosSubscriberTwistStamped(const RosSubscriberTwistStamped&) = delete;
  RosSubscriberTwistStamped& operator=(const RosSubscriberTwistStamped&) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   *
   * @param msg - message content
   */
  void parseMsg(const TRosMsg& msg) override {
    this->getOut().getSignal().setValue({msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                              msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z});
    this->getOut().getSignal().setTimestamp(RosTools::convertToEerosTime(msg.header.stamp));
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, const RosSubscriberTwistStamped& s) {
  os << "Block RosSubscriberTwistStamped: '" << s.getName() << "'";
  return os;
}

}
