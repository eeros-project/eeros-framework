#pragma once

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/control/ros2/EerosRosTools.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>

#include <geometry_msgs/msg/twist.hpp>


namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This block subscribes a Twist msg
 */
typedef geometry_msgs::msg::Twist ros2_msg_t;
class Ros2SubscriberTwist : public RosSubscriber<ros2_msg_t, math::Vector3>{
 public:

  /**
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  Ros2SubscriberTwist(const rclcpp::Node::SharedPtr node, 
                       const std::string& topic, 
                       const uint32_t queueSize=1000)
    : RosSubscriber<ros2_msg_t, math::Vector3>(node, topic, false, queueSize){
    CHECK_RCLCPP_OK();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Ros2SubscriberTwist(const Ros2SubscriberTwist&) = delete;

  virtual void parseMsg(const ros2_msg_t& msg) override {
    out.getSignal().setValue({msg.linear.x, msg.linear.y, msg.linear.z});
    out.getSignal().setTimestamp(eeros::System::getTimeNs());
    outAngular.getSignal().setValue({msg.angular.x, msg.angular.y, msg.angular.z});
    outAngular.getSignal().setTimestamp(eeros::System::getTimeNs());
  }

   /**
   * Get rotation of a twist message
   * 
   * @return input
   */
  virtual Output<math::Vector3>& getOutAngular() {return outAngular;}

 private:
  Output<math::Vector3> outAngular;
};

} /* END: Namespace control */
} /* END: Namespace eeros */