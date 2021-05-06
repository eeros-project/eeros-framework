#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP

#include <eeros/control/ros/RosSubscriber.hpp>
#include <eeros/core/System.hpp>
#include <std_msgs/Float64MultiArray.h>

namespace eeros {
namespace control {

/**
 * This block allows to subscribe to a ROS message of type std_msgs::Float64MultiArray::Type
 * and publishes it as a signal of type Matrix<N,1,double>.
 * 
 * @tparam SigOutType - type of the output signal
 * @since v1.0
 */
template < typename SigOutType >
class RosSubscriberDoubleArray : public RosSubscriber<std_msgs::Float64MultiArray::Type, SigOutType> {
  typedef std_msgs::Float64MultiArray::Type TRosMsg;
  
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages of type std_msgs::Float64MultiArray::Type under a given topic 
   * and outputs its values onto a signal of type Matrix<N,1,double>. 
   * If several messages are pending for a given topic
   * you can choose if the block simply consumes the oldest message or 
   * processes all pending messages.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   * @param callNewest - set to true if all pending messages should be processed
   */
  RosSubscriberDoubleArray(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false ) 
      : RosSubscriber<TRosMsg, SigOutType>(topic, queueSize, callNewest) { }
    
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
  void rosCallbackFct(const TRosMsg& msg) {
    auto time = eeros::System::getTimeNs();	// use system time for timestamp
    this->out.getSignal().setTimestamp( time );
    std::vector<double> valTmp(msg.data.begin(), msg.data.end() );
    val.setCol(0, valTmp);
    this->out.getSignal().setValue(val);
  }
  
 protected:
  SigOutType val;
};

}
}

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_DOUBLEARRAY_HPP
