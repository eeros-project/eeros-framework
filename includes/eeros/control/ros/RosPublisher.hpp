#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <eeros/control/Block1i.hpp>

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
class RosPublisher : public Block1i<SigInType> {
 public:
  /**
   * Creates an instance of a ROS publisher block. The block reads
   * the signal on its input and publishes it under a given topic name.
   * If no ROS master can be found, the block does not do anything.
   * 
   * @param node_name - name of this node
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   * @param callNewest - not used
   */
  RosPublisher(const std::string& node_name, const std::string& topic, uint32_t queueSize=1000, bool callNewest=false)
      : topic(topic) {
    if (rclcpp::ok()) {
      handle = rclcpp::Node::make_shared(node_name);
      publisher = handle->create_publisher<TRosMsg>(topic, queueSize);
      RCLCPP_DEBUG_STREAM(handle->get_logger(), "RosBlockPublisher, reading from topic: '" << topic << "' on node '" << node_name << "' created.");
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
    rclcpp::Node::SharedPtr handle;
    typename rclcpp::Publisher<TRosMsg>::SharedPtr publisher;
    const std::string& topic;
    TRosMsg msg;
    bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_HPP_ */
