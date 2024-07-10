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
   * 
   * @throw std::runtime_error if rclcpp not initialized
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
    lastTime = 0;
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Ros2PublisherOdometrie(const Ros2PublisherOdometrie& other) = delete;

  virtual void setRosMsg(nav_msgs::msg::Odometry& msg) override {
    this->isNaN = false;
    msg.header.frame_id = odomFrame;
    msg.header.stamp = clock->now();
    msg.child_frame_id = robotBaseFrame;
    temp = inTwistTranslation.getSignal().getValue();
    hasNaN(temp);
    msg.twist.twist.linear.x = temp[0];
    msg.twist.twist.linear.y = temp[1];
    msg.twist.twist.linear.z = temp[2];
    temp = inTwistRotation.getSignal().getValue();
    hasNaN(temp);
    msg.twist.twist.angular.x = temp[0];
    msg.twist.twist.angular.y = temp[1];
    msg.twist.twist.angular.z = temp[2];
    temp = inPoseTranslation.getSignal().getValue();
    hasNaN(temp);
    msg.pose.pose.position.x = temp[0];
    msg.pose.pose.position.y = temp[1];
    msg.pose.pose.position.z = temp[2];
    temp = inPoseRotation.getSignal().getValue();
    hasNaN(temp);
    quat.setRPY(temp[0], temp[1], temp[2]);
    msg.pose.pose.orientation.x = quat.getX();
    msg.pose.pose.orientation.y = quat.getY();
    msg.pose.pose.orientation.z = quat.getZ();
    msg.pose.pose.orientation.w = quat.getW();
    hasNaN(inCovarianzTwist.getSignal().getValue());
    hasNaN(inCovarianzPose.getSignal().getValue());
    msg.twist.covariance = inCovarianzTwist.getSignal().getValue();
    msg.pose.covariance = inCovarianzPose.getSignal().getValue();
  }

  /**
   * Override to avoid publish if a Value is NaN
   */
  virtual void run() override{
    if (running) {    
      setRosMsg(msg);
      /* if a value is NaN do not publish it */
      if(this->isNaN) {
        if((lastTime + 2000000000) < eeros::System::getTimeNs()){
          log.warn() << "Odom block: \"" << this->getName() << "\"";
          log.warn() << " -> It has not been published due to NaN value(s).";
          lastTime = eeros::System::getTimeNs();
        }
      } else {
        publisher->publish(msg);
      }
    }
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
  bool isNaN = false;
  uint64_t lastTime;

  std::string robotBaseFrame;
  std::string odomFrame;
  rclcpp::Clock::SharedPtr clock;

  void hasNaN(math::Vector3 x){
    if(std::isnan(x[0]+x[1]+x[2])) {
      isNaN = true;
    }
  }

  void hasNaN(const std::array<double, 36>& x) {
    double res = 0;
    for (const auto& value : x) {
      res += value;
    }
    if (std::isnan(res)) {
      isNaN = true;
    }
  }
};

} /* END: Namespace control */
} /* END: Namespace eeros */