#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP

#include <eeros/control/ros/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <std_msgs/Float64.h>

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type std_msgs::Float64::Type
 * and publishes it as a signal of type double.
 * 
 * @since v1.0
 */
class RosSubscriberDouble : public RosSubscriber<std_msgs::Float64::Type, double> {
  typedef std_msgs::Float64::Type TRosMsg;
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type std_msgs::Float64::Type under a given topic 
   * and outputs its values onto a signal of type double. 
   * If several messages are pending for a given topic
   * you can choose if the block simply consumes the oldest message or 
   * processes all pending messages.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param topic - name of the topic
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   * @param callNewest - set to true if all pending messages should be processed
   */
  RosSubscriberDouble(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false ) 
      : RosSubscriber<TRosMsg, double>(topic, queueSize, callNewest) { }
    
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosSubscriberDouble(const RosSubscriberDouble& other) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   * 
   * @param msg - message content
   */
  virtual void rosCallbackFct(const TRosMsg& msg) {
    auto time = eeros::System::getTimeNs();	// use system time for timestamp
    this->out.getSignal().setTimestamp( time );
    this->out.getSignal().setValue(static_cast<double>(msg.data) );
  }
};

}
}

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLE_HPP
