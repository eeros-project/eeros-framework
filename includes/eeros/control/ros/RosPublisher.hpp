#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

namespace eeros {
namespace control {

/**
 * This is the base class for all blocks which publish ROS messages.
 * 
 * @tparam TRosMsg - type of the ROS message
 * @tparam SigInType - type of the input signal
 * @since v1.0
 */
template < typename TRosMsg, typename SigInType >
class RosPublisher : public Blockio<1, 0, SigInType> {
 public:
  /**
   * Creates an instance of a ROS publisher block. The block reads
   * the signal on its input and publishes it under a given topic name.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param node - The ROS Node as a SharedPtr
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   * @param callNewest - not used
   */
  RosPublisher(const rclcpp::Node::SharedPtr node, const std::string& topic, uint32_t queueSize=1000, bool callNewest=false)
      : handle(node),
        topic(topic),
        log(logger::Logger::getLogger()) {
    if (rclcpp::ok()) {
      publisher = handle->create_publisher<TRosMsg>(topic, queueSize);
      log.info() << "RosBlockPublisher, writing to topic: '" << topic << "' on node '" << node->get_name() << "' created.";
      running = true;
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisher(const RosPublisher& other) = delete;

  /**
   * Sets the message to be published by this block.
   * 
   * @param msg - message content
   */
  virtual void setRosMsg(TRosMsg& msg) = 0;
  
  /**
   * This method will be executed whenever the block runs.
   */
  virtual void run() {
    if (running) {
      setRosMsg(msg);
      publisher->publish(msg);
    }
  }

  /**
   * This logger is used to put out information about the safety system
   */
  logger::Logger log;


  protected:
    rclcpp::Node::SharedPtr handle;
    typename rclcpp::Publisher<TRosMsg>::SharedPtr publisher;
    const std::string& topic;
    TRosMsg msg;
    bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_ */
