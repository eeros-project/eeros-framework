#pragma once

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros/control/ros2/EerosRosTools.hpp>
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

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)


/**
 * This block will creates a odom message from the input signals and publishes it.
 * Pose -> Describes the position of the robot since the start.
 */
class Ros2PublisherOdometrie : public RosPublisher<nav_msgs::msg::Odometry, uint8_t>{
 public:

  /**
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  Ros2PublisherOdometrie(const rclcpp::Node::SharedPtr node, 
                         const std::string& topic,
                         std::string robotBaseFrame,
                         std::string odomFrame,
                         const uint32_t queueSize=1000)
    : RosPublisher<nav_msgs::msg::Odometry, uint8_t>(node, topic, queueSize),
      robotBaseFrame(robotBaseFrame),
      odomFrame(odomFrame),
      clock(node->get_clock()) { 
    CHECK_RCLCPP_OK();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Ros2PublisherOdometrie(const Ros2PublisherOdometrie& other) = delete;

  virtual void setRosMsg(nav_msgs::msg::Odometry& msg) override {
    msg.header.frame_id = odomFrame;
    msg.header.stamp = clock->now();
    msg.child_frame_id = robotBaseFrame;
    msg.twist.twist.linear.x = inTwistTranslation.getSignal().getValue()[0];
    msg.twist.twist.linear.y = inTwistTranslation.getSignal().getValue()[1];
    msg.twist.twist.linear.z = inTwistTranslation.getSignal().getValue()[2];
    msg.twist.twist.angular.z = inTwistRotation.getSignal().getValue()[0];
    msg.twist.twist.angular.x = inTwistRotation.getSignal().getValue()[1];
    msg.twist.twist.angular.y = inTwistRotation.getSignal().getValue()[2];
    msg.pose.pose.position.x = inPoseTranslation.getSignal().getValue()[0];
    msg.pose.pose.position.y = inPoseTranslation.getSignal().getValue()[1];
    msg.pose.pose.position.z = inPoseTranslation.getSignal().getValue()[2];
    temp = inPoseRotation.getSignal().getValue();
    quat.setRPY(temp[0], temp[1], temp[2]);
    msg.pose.pose.orientation.x = quat.getX();
    msg.pose.pose.orientation.y = quat.getY();
    msg.pose.pose.orientation.z = quat.getZ();
    msg.pose.pose.orientation.w = quat.getW();
    msg.twist.covariance = inCovarianzTwist.getSignal().getValue();
    msg.pose.covariance = inCovarianzPose.getSignal().getValue();
  }

  inline control::Input<math::Vector3>& getInPoseTranslation(){return inPoseTranslation;}
  inline control::Input<math::Vector3>& getInPoseRotation(){return inPoseRotation;}
  inline control::Input<math::Vector3>& getInTwistTranslation(){return inTwistTranslation;}
  inline control::Input<math::Vector3>& getInTwistRotation(){return inTwistRotation;}
  inline control::Input<std::array<double,36>>& getInCovarPose(){return inCovarianzPose;}
  inline control::Input<std::array<double,36>>& getInCovarTwist(){return inCovarianzTwist;}

 private:
  control::Input<math::Vector3> inPoseTranslation;
  control::Input<math::Vector3> inPoseRotation;
  control::Input<math::Vector3> inTwistTranslation;
  control::Input<math::Vector3> inTwistRotation;
  control::Input<std::array<double,36>> inCovarianzPose;
  control::Input<std::array<double,36>> inCovarianzTwist;
  math::Vector3 temp;
  tf2::Quaternion quat;

  std::string robotBaseFrame;
  std::string odomFrame;
  rclcpp::Clock::SharedPtr clock;
};

} /* END: Namespace control */
} /* END: Namespace eeros */