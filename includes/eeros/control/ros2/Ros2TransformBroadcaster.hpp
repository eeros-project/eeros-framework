#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>

#include <eeros/control/Block.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/control/ros2/EerosRosTools.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is used to publish a tf frame to the global tf tree.
 * 
 * @tparam N defines how many publication to the tf tree
 * @tparam N Must fullfill: N == frames.size
 * 
 * This is only for dynamic transformations.
 * For static transformation see class: eeros::control::Ros2StaticTransformBroadcaster.hpp
 */
template<uint8_t N>
class Ros2TransformBroadcaster : public Block {
 public:

  /**
   * Creates a tf listener for dynamic tf's
   * 
   * @param node: a rclcpp node (dosent depend whitch one. Only for Ros clock needed)
   * @param frames: first element parent second element child frame
   *                Index n belongs to output n
   * @param queueSize: rclcpp QoS 
   * 
   * @throw std::runtime_error if rclcpp is not initialized
   * @throw eeros::Fault if frames vector size unequal to template N
   */
  Ros2TransformBroadcaster(rclcpp::Node::SharedPtr node,
                           const std::vector<std::pair<std::string, std::string>>& frames_, 
                           uint32_t queueSize=1000)
    : running(false),
      node(node),
      frames(frames_),
      log(logger::Logger::getLogger('t')) {
    static uint8_t objectCount = 0;
    static std::mutex mtx;
    if(frames.size() != N){
        std::ostringstream oss;
        oss << __FILE__ << "::" << __LINE__ << "\n frames (size: " << frames.size() << ") must have the same length as template param N=" << N;
        throw eeros::Fault(oss.str());
    }
    for(int i = 0; i<N; i++){
      lastErrTime[i] = 0;
      if(frames[i].first.compare(frames[i].second) == 0) {
        log.warn() << "Parent and child frame are the same. Parent: '" << frames[i].first <<"' Child: '" << frames[i].second << "'";
      }
    }
    
    /* Checker to warn if more than one instance is created */
    mtx.lock();
    if(objectCount != 0){
      log.warn() << "Created multiple instance of class Ros2TransformBroadcaster.";
      log.warn() << " -> It is recommended to create only one instance!!!";
    }
    objectCount++;
    mtx.unlock();
    
    CHECK_RCLCPP_OK();
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node, queueSize);
    log.info() << "Ros tf broadcaster generated ' on node '" << node->get_name();
   
    running = true;
  }

  virtual void run() {
    if(running) {
      for(uint8_t i = 0; i<N;i++){
        inTrans = inPoseTranslation[i].getSignal().getValue();
        inRot = inPoseRotation[i].getSignal().getValue();
        /* if one number is nan skip */
        if(this->hasNaN(inTrans) || this->hasNaN(inRot)){
          if(lastErrTime[i] + 2000000000 < System::getTimeNs()){
            log.warn() << "Couldent pubish tf: '" << frames[i].first << "' to '" << frames[i].second << "' due to NaN in values";
            lastErrTime[i] = System::getTimeNs();
          }
          continue;
        }
        msg.child_frame_id = frames[i].second;
        msg.header.frame_id = frames[i].first;
        msg.header.stamp = node->get_clock()->now();
        msg.transform.translation.x = inTrans[0];
        msg.transform.translation.y = inTrans[1];
        msg.transform.translation.z = inTrans[2];
        quaternion.setRPY(inRot[0], inRot[1], inRot[2]);
        msg.transform.rotation.x = quaternion.x();
        msg.transform.rotation.y = quaternion.y();
        msg.transform.rotation.z = quaternion.z();
        msg.transform.rotation.w = quaternion.w();
        tf_broadcaster->sendTransform(msg);
      }
    }
  }
  
  /**
   * Get the input for translation
   * 
   * @param index Index has the same function as getInPoseRotation. It belongs to a tf
   * @return the Input
   * 
   * @throw eeros::control::IndexOutOfBoundsFault if index >= N
   */
  inline control::Input<math::Vector3>& getInPoseTranslation(const uint8_t index){
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return inPoseTranslation[index];
  }

  /**
   * Get the input for rotation
   * 
   * @param index Index has the same function as getInPoseTranslation. It belongs to a tf
   * @return the Input
   * 
   * @throw eeros::control::IndexOutOfBoundsFault if index >= N
   */
  inline control::Input<math::Vector3>& getInPoseRotation(const uint8_t index){
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return inPoseRotation[index];
  }

 protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  bool running;
  eeros::logger::Logger log;
  geometry_msgs::msg::TransformStamped msg;
  const std::vector<std::pair<std::string, std::string>> frames;
  control::Input<math::Vector3> inPoseTranslation[N];
  control::Input<math::Vector3> inPoseRotation[N];
  math::Vector3 inTrans;
  math::Vector3 inRot;
  uint64_t lastTime = 0;
  tf2::Quaternion quaternion;
  uint64_t lastErrTime[N];

 private:
  bool hasNaN(math::Vector3 x){
    return std::isnan(x[0]+x[1]+x[2]);
  }
};

} /* END Namespace: control */
} /* END Namespace: eeros */