#ifndef ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_
#define ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {
  
/**
 * This is the base class for all blocks which subscribe to ROS messages.
 * 
 * @tparam TRosMsg - type of the ROS message
 * @tparam SigOutType - type of the input signal
 * @since v1.0
 */
template < typename TRosMsg, typename SigOutType >
class RosSubscriber : public Block1o<SigOutType> {
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages under a given topic and outputs its values onto
   * a signal output. If several messages are pending for a given topic
   * you can choose if the block simply consumes the oldest message or 
   * processes all pending messages.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param topic - name of the topic
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   * @param callNewest - set to true if all pending messages should be processed
   */
  RosSubscriber(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false) 
      : topic(topic), callNewest(callNewest) {
    if (ros::master::check()) { 
      ros::NodeHandle handle;
      subscriber = handle.subscribe(topic, queueSize, &RosSubscriber::rosCallbackFct, this);
      ROS_DEBUG_STREAM("RosBlockSubscriber, reading from topic: '" << topic << "' created.");
      running = true;
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosSubscriber(const RosSubscriber& other) = delete;

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   * 
   * @param msg - message content
   */
  virtual void rosCallbackFct(const TRosMsg& msg) = 0;

// 	void rosCallbackFct(const TRosMsg& msg) {
// 
// 	// USER DEFINED: 1.) Set timestamp for all outputs
// 	//               2.) Get the data from the message
// 	//               3.) Cast the data if necessary
// 	//               4.) Insert the data into output
// 
// 	auto time = eeros::System::getTimeNs();
// 	this->out.getSignal().setTimestamp( time );
// 
// 	this->out.getSignal().setValue(static_cast< TOutput >( msg.data) );
// 	}

  /**
   * This method will be executed whenever the block runs.
   */
  virtual void run() {
    if (running) {  
      if (callNewest)
        ros::getGlobalCallbackQueue()->callAvailable();	// calls callback fct. for all available messages.
      else
        ros::getGlobalCallbackQueue()->callOne();		// calls callback fct. only for the oldest message
    }
  }

 protected:
  ros::Subscriber subscriber;
  const std::string& topic;
  bool callNewest;
  bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_ */
