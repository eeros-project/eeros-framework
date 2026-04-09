
#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/math/Matrix.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace eeros::control {

/**
 * This block reads an input signal and publishes it as a twist message
 * of type 'geometry_msgs/TwistStamped'.
 */
class RosPublisherTwistStamped : public RosPublisher<geometry_msgs::msg::TwistStamped, 1, eeros::math::Matrix<6>> {

  using TRosMsg = geometry_msgs::msg::TwistStamped;
  using Base = RosPublisher<TRosMsg, 1, eeros::math::Matrix<6>>;

 public:

  /**
   * Creates an instance of a publisher block which publishes several input signals
   * as a ROS message of type geometry_msgs::msg::TwistStamped.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisherTwistStamped(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000)
      : Base(node, topic, queueSize) { }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosPublisherTwistStamped(const RosPublisherTwistStamped& other) = delete;
  RosPublisherTwistStamped& operator=(const RosPublisherTwistStamped&) = delete;

  /**
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  void setRosMsg(TRosMsg& msg) override {
    msg.header.frame_id = " ";
    msg.header.stamp = RosTools::convertToRosTime(this->getIn().getSignal().getTimestamp());

    auto temp = this->getIn().getSignal().getValue();
    msg.twist.linear.x = temp[0];
    msg.twist.linear.y = temp[1];
    msg.twist.linear.z = temp[2];
    msg.twist.angular.x = temp[3];
    msg.twist.angular.y = temp[4];
    msg.twist.angular.z = temp[5];
  }

};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, const RosPublisherTwistStamped& p) {
  os << "Block RosPublisherTwistStamped: '" << p.getName() << "'";
  return os;
}

}
