#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <eeros/core/System.hpp>
#include <array>

namespace eeros {
namespace control {

using namespace eeros::math;

/**
 * This block creates a odometry message of type 'nav_msgs/odometry' from the input signals
 * 'pose' and 'twist' and publishes it as a 'nav_msgs::msg::Odometry' message.
 * 'pose' is of type 'geometry_msgs/PoseWithCovariance'
 * 'twist' is of type 'geometry_msgs/TwistWithCovariance'
 * The block has 4 input vectors of type 'Vector3' with
 * - index 0: pose translation
 * - index 1: pose rotation
 * - index 2: twist translation
 * - index 3: twist rotation
 * and two further input vectors for the covariances for pose and twist.
 *
 * @since v1.4
 */
class RosPublisherOdometry : public RosPublisher<nav_msgs::msg::Odometry, 4, Vector3>{
 public:

  /**
   * Creates an instance of a publisher block which publishes several input signals
   * as a ROS message of type nav_msgs::msg::Odometry.
   *
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param rbf - name of the child frame id
   * @param of - name of odometry frame
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisherOdometry(const rclcpp::Node::SharedPtr node, const std::string& topic, std::string rbf, std::string of, const uint32_t queueSize=1000)
      : RosPublisher<nav_msgs::msg::Odometry, 4, Vector3>(node, topic, queueSize), robotBaseFrame(rbf), odomFrame(of), clock(node->get_clock()) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherOdometry(const RosPublisherOdometry& other) = delete;

  /**
   * Sets the message to be published by this block.
   *
   * @param msg - message content
   */
  virtual void setRosMsg(nav_msgs::msg::Odometry& msg) override {
    msg.header.frame_id = odomFrame;
    msg.header.stamp = clock->now();
    msg.child_frame_id = robotBaseFrame;
    Vector3 temp = in[0].getSignal().getValue();
    msg.pose.pose.position.x = temp[0];
    msg.pose.pose.position.y = temp[1];
    msg.pose.pose.position.z = temp[2];

    temp = in[1].getSignal().getValue();
    tf2::Quaternion quat;
    quat.setRPY(temp[0], temp[1], temp[2]);
    msg.pose.pose.orientation.x = quat.getX();
    msg.pose.pose.orientation.y = quat.getY();
    msg.pose.pose.orientation.z = quat.getZ();
    msg.pose.pose.orientation.w = quat.getW();

    temp = in[2].getSignal().getValue();
    msg.twist.twist.linear.x = temp[0];
    msg.twist.twist.linear.y = temp[1];
    msg.twist.twist.linear.z = temp[2];

    temp = in[3].getSignal().getValue();
    msg.twist.twist.angular.x = temp[0];
    msg.twist.twist.angular.y = temp[1];
    msg.twist.twist.angular.z = temp[2];

    msg.pose.covariance = inCovarianzPose.getSignal().getValue();
    msg.twist.covariance = inCovarianzTwist.getSignal().getValue();
  }

  /**
   * Getter function for input for the covariances of the pose
   *
   * @return input
   */
  inline Input<std::array<double,36>>& getInCovarPose(){return inCovarianzPose;}

  /**
   * Getter function for input for the covariances of the twist
   *
   * @return input
   */
  inline Input<std::array<double,36>>& getInCovarTwist() {return inCovarianzTwist;}

 private:
  Input<std::array<double,36>> inCovarianzPose;
  Input<std::array<double,36>> inCovarianzTwist;
  std::string robotBaseFrame;
  std::string odomFrame;
  rclcpp::Clock::SharedPtr clock;
};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, RosPublisherOdometry& p) {
  os << "Block RosPublisherOdometry: '" << p.getName();
  return os;
}

}
}
