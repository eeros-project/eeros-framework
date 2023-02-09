#ifndef ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_
#define ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/control/ros2/EerosRosTools.hpp>
#include <deque>
#include <thread>

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
class RosSubscriber : public Blockio<0,1,SigOutType> {
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages under a given topic and outputs its values onto
   * a signal output. If several messages are pending for a given topic
   * you can choose if the block simply consumes the oldest message or 
   * processes all pending messages.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   * @param callNewest - set to true if all pending messages should be processed
   */
  RosSubscriber(const rclcpp::Node::SharedPtr node, const std::string& topic, const uint32_t queueSize=1000)
      : node(node), t(std::thread(&RosSubscriber::spin,this)), log(logger::Logger::getLogger()) {
    if (rclcpp::ok()) {
 //     rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
 //     options.callback_group = Executor::instance().registerSubscriber(handle, syncWithTopic);
      subscriber = node->create_subscription<TRosMsg>(topic, queueSize, std::bind(&RosSubscriber::rosSubscriberCallback, this, _1));
      RCLCPP_INFO_STREAM(node->get_logger(), "RosBlockSubscriber, reading from topic: '" << topic << "' created.");
//      log.warn() << "RosBlockSubscriber, reading from topic: '" << topic << "' created.";
      running = true;
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosSubscriber(const RosSubscriber& other) = delete;

  /**
   * This is the callback which is called on every message the subscriber receives.
   * The queue is afterwards worked by the run method which is called form the EEROS-Executor.
   */
  void rosSubscriberCallback(const TRosMsg& msg) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    queue.push_back(std::move(msg));
  }

  void spin() {
    while(!running);
    eeros::control::rosTools::Spinner::getInstance().startSpinning(node);
  }

  /**
   * This function is called whenever the run function reads the
   * next pending ROS message.
   * 
   * @param msg - message content
   */
  virtual void parseMsg(const TRosMsg& msg) = 0;

  /**
   * This method will be executed whenever the block runs.
   */
  virtual void run() {
    if (running && !queue.empty()) {
      std::lock_guard<std::mutex> lock(queue_mutex);
      TRosMsg msg = std::move(queue.back());
      parseMsg(msg);
      queue.clear();
    }
  }

 protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Subscription<TRosMsg>::SharedPtr subscriber;
  bool running = false;
  // thread-safe queue which is filled in callback function called by ROS::Executor
  // emptied by handled by run method called by EEROS::Executor
  std::deque<TRosMsg> queue;
  std::mutex queue_mutex;
  std::thread t;
  logger::Logger log;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSSUBCRIBER_HPP_ */
