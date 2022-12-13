#ifndef ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_
#define ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
namespace control {
  
  using std::placeholders::_1;

/**
 * This is the base class for all blocks which subscribe to ROS messages.
 * 
 * @tparam TRosMsg - type of the ROS message
 * @tparam SigOutType - type of the input signal
 * @since v1.0
 */
template < typename TRosMsg, typename SigOutType >
class RosSubscriber : public Blockio<0, 1, SigOutType> {
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages under a given topic and outputs its values onto
   * a signal output. If several messages are pending for a given topic
   * you can choose if the block simply consumes the oldest message or 
   * processes all pending messages.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param node - ROS Node as a SharedPtr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   * @param callNewest - not used anymore
   */
  RosSubscriber(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000, bool callNewest=false)
      : handle(node),
        topic(topic),
        log(logger::Logger::getLogger()) {
    if (rclcpp::ok()) {
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
      options.callback_group = Executor::instance().registerSubscriberNode(handle);
      subscriber = handle->create_subscription<TRosMsg>(topic, queueSize, std::bind(&RosSubscriber::rosCallbackFct, this, _1), options);
      log.info() << "RosBlockSubscriber, reading from topic: '" << topic << "' on node '" << node->get_name() << "' created.";
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
      /*if (callNewest) {
        // calls callback fct. for all available messages.
        //Executor::instance().spin_some();
      } else {
        // calls callback fct. only for the oldest message
        //Executor::instance().spin();
      }*/
    }
  }

  /**
   * This logger is used to put out information about the safety system
   */
  logger::Logger log;

 protected:
  rclcpp::Node::SharedPtr handle;
  typename rclcpp::Subscription<TRosMsg>::SharedPtr subscriber;
  const std::string& topic;
  bool callNewest;
  bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_ */
