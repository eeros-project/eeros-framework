#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_
#define ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_

#include <eeros/control/ros/RosPublisher.hpp>
#include <std_msgs/UInt32.h>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/Block.hpp>

using namespace eeros::safety;

namespace eeros {
namespace control {

/**
 * This block allows to publish the safety level of the safety SafetySystem
 * as a ROS message of type std_msgs::UInt32::Type.
 * 
 * @since v1.0
 */
class RosPublisherSafetyLevel : public Block {
  typedef std_msgs::UInt32::Type TRosMsg;
  
 public:
  /**
   * Creates an instance of a publisher block which publishes the safety level.
   * 
   * @param topic - name of the topic
   * @param queueSize - maximum number of outgoing messages to be queued for delivery to subscribers
   */ 
  RosPublisherSafetyLevel(const std::string& topic, const uint32_t queueSize=1000) 
      : topic(topic) {
    if (ros::master::check()) {    
      ros::NodeHandle handle;
      publisher = handle.advertise<TRosMsg>(topic, queueSize);
      ROS_DEBUG_STREAM("RosPublisherSafetyLevel, reading from topic: '" << topic << "' created.");
      running = true;
    }
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RosPublisherSafetyLevel(const RosPublisherSafetyLevel& other) = delete;

  /**
   * Stores a reference to the safety system. Call this function after creating the 
   * control system and the safety system.
   * 
   * @param ss - safety system
   */
  void setSafetySystem(SafetySystem& ss) {
    safetySystem = &ss;
  }
  
  /**
   * This method will be executed whenever the block runs.
   */
  virtual void run() {
    if (running && safetySystem != nullptr) {
      SafetyLevel sl = safetySystem->getCurrentLevel();
      msg.data = sl.getLevelId();
      publisher.publish(msg);
    }
  }
  
 protected:
  ros::Publisher publisher;
  const std::string& topic;
  TRosMsg msg;
  SafetySystem* safetySystem;
  bool running = false;
};

}
}

#endif /* ORG_EEROS_CONTROL_ROSPUBLISHER_SAFETYLEVEL_HPP_ */
