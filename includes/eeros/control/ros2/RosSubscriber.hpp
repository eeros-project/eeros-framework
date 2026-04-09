#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <deque>
#include <mutex>
#include <functional>

namespace eeros::control {
  
/**
 * @brief Base class for blocks that subscribe to a ROS2 topic.
 *
 * Receives ROS messages asynchronously via a ROS callback, queues them
 * thread-safely, and processes them on the next @ref run() call from the
 * EEROS executor. If multiple messages arrive between two @ref run() calls,
 * only the most recent is processed and the queue is cleared.
 *
 * Subclasses must implement @ref parseMsg() to extract signal values from
 * the incoming message type.
 * 
 * @tparam TRosMsg - type of the ROS message
 * @tparam M - number of outputs
 * @tparam SigOutType - type of the output signal

 * @since v1.0
 */
template < typename TRosMsg, uint8_t M, typename SigOutType >
class RosSubscriber : public Blockio<0,M,SigOutType> {
 public:
  /**
   * Creates an instance of a ROS subscriber block. The block reads
   * ROS messages under a given topic and outputs its values onto
   * a signal output. If several messages are pending for a given topic
   * all the messages are consumed and the signal is set to the
   * newest.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param syncWithTopic - when set to true the executor runs all time domains upon receiving this message
   * @param queueSize - maximum number of incoming messages to be queued for delivery to subscribers
   */
  RosSubscriber(const rclcpp::Node::SharedPtr node, const std::string& topic, bool syncWithTopic=false, const uint32_t queueSize=1000)
      : node(std::move(node)), sync(syncWithTopic), log(eeros::logger::Logger::getLogger()) {
  if (!rclcpp::ok()) return;
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(queueSize);
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = Executor::instance().registerSubscriber(this->node, syncWithTopic);
  subscriber = this->node->create_subscription<TRosMsg>(topic, qos, [this](const TRosMsg& msg) { rosSubscriberCallback(msg); }, options);
  log.info() << "RosBlockSubscriber, reading from topic: '" << topic << "' on node '" << node->get_name();
  running = true;
}

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosSubscriber(const RosSubscriber&) = delete;
  RosSubscriber& operator=(const RosSubscriber&) = delete;

  /**
   * @brief ROS callback — queues the incoming message thread-safely.
   *
   * Called from the ROS executor thread. If @c syncWithTopic is set,
   * also triggers the EEROS executor.
   *
   * @param msg  Incoming ROS message
   */
  void rosSubscriberCallback(const TRosMsg& msg) {
    std::lock_guard lock(queueMutex);
    queue.push_back(std::move(msg));
    if (sync) {
      Executor::instance().handleTopic();
    }
  }

  /**
   * @brief Parses a received ROS message and writes values to the output signals.
   *
   * Called by @ref run() for the most recent queued message.
   * Must be implemented by subclasses.
   *
   * @param msg  The most recently received message
   */
  virtual void parseMsg(const TRosMsg& msg) = 0;

  /**
   * This method will be executed whenever the block runs.
   */
  void run() override {
    if (running && !queue.empty()) {
      std::unique_lock lock(queueMutex);
      TRosMsg msg = std::move(queue.back());
      parseMsg(msg);
      queue.clear();
    }
  }

 protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Subscription<TRosMsg>::SharedPtr subscriber;
  bool running{false};
  bool sync;
  // thread-safe queue which is filled in callback function called by ROS::Executor
  // emptied by handled by run method called by EEROS::Executor
  std::deque<TRosMsg> queue;
  std::mutex queueMutex;
  eeros::logger::Logger log;
};

}
