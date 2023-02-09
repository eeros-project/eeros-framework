#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/logger/Logger.hpp>

using namespace eeros::logger;

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
class RosPublisher : public Blockio<1,0,SigInType> {
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
      : log(Logger::getLogger()) {
    if (rclcpp::ok()) {
      publisher = node->create_publisher<TRosMsg>(topic, queueSize);
      log.info() << "RosBlockPublisher, writing to topic: '" << topic << "' on node '" << node->get_name() << "' created.";
      RCLCPP_INFO_STREAM(node->get_logger(), "RosBlockPublisher to topic: '" << topic << "' created.");
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


 protected:
  typename rclcpp::Publisher<TRosMsg>::SharedPtr publisher;
  TRosMsg msg;
  bool running = false;
  Logger log;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_ */
