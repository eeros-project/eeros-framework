#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/logger/Logger.hpp>

namespace eeros::control {

/**
* @brief Base class for blocks that publish signal values as ROS2 messages.
 *
 * On each @ref run() call, @ref setRosMsg() is invoked to populate a message
 * from the block's input signals, which is then published on the configured topic.
 * If ROS is not running at construction time, the block silently does nothing.
 *
 * Subclasses must implement @ref setRosMsg() to fill the message from inputs.
 * 
 * @tparam TRosMsg - type of the ROS message
 * @tparam N - number of inputs
 * @tparam SigInType - type of the input signal
 *
 * @since v1.0
 */
template < typename TRosMsg, uint8_t N, typename SigInType >
class RosPublisher : public Blockio<N,0,SigInType> {
 public:
  /**
   * Creates an instance of a ROS publisher block. The block reads
   * the signal on its input and publishes it under a given topic name.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param node - ROS node as a shared ptr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */
  RosPublisher(const rclcpp::Node::SharedPtr node, const std::string& topic, uint32_t queueSize=1000)
      : log(eeros::logger::Logger::getLogger()) {
    if (!rclcpp::ok()) return;
    publisher = node->create_publisher<TRosMsg>(topic, queueSize);
    log.info() << "RosBlockPublisher, writing to topic: '" << topic << "' on node '" << node->get_name();
    running = true;
  }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  RosPublisher(const RosPublisher& other) = delete;
  RosPublisher& operator=(const RosPublisher&) = delete;

  /**
   * Sets the message to be published by this block.
   * 
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) = 0;
  
  /**
   * This method will be executed whenever the block runs.
   */
  void run() override {
    if (!running) return;   
    setRosMsg(msg);
    publisher->publish(msg);
  }

 protected:
  typename rclcpp::Publisher<TRosMsg>::SharedPtr publisher;
  TRosMsg msg{};
  bool running{false};
  eeros::logger::Logger log;
};

}
