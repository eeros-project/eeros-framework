#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP
#define ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP

#include <eeros/control/ros2/RosPublisher.hpp>
#include <eeros/math/Matrix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace eeros {
namespace control {

/**
 * This block allows to read a single input signal of type Matrix<N,1,double> and
 * publishes it as a ROS message of type std_msgs::Float64MultiArray.
 * 
 * @tparam SigInType - type of the input signal
 * @since v1.0
 *
 * @deprecated Since Foxy
 */
template < typename SigInType >
class RosPublisherDoubleArray : public RosPublisher<std_msgs::msg::Float64MultiArray, SigInType> {
  typedef std_msgs::msg::Float64MultiArray TRosMsg;
  
 public:
  /**
   * Creates an instance of a publisher block which publishes a input signal 
   * of type Matrix<N,1,double> as a ROS message of type std_msgs::msg::Float64MultiArray.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  RosPublisherDoubleArray(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000) :
    RosPublisher<TRosMsg, SigInType>(node, topic, queueSize) { }
    
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherDoubleArray(const RosPublisherDoubleArray& other) = delete;

  /**
   * Sets the message to be published by this block.
   * 
   * @param msg - message content
   */
  void setRosMsg(TRosMsg& msg) {
    if (this->in.isConnected()) {
      auto val = this->in.getSignal().getValue();
      auto valTmpDouble = val.getColVector(0);
      msg.data = valTmpDouble;
    }
  }
};

}
}

#endif // ORG_EEROS_CONTROL_ROSPUBLISHER_DOUBLEARRAY_HPP
