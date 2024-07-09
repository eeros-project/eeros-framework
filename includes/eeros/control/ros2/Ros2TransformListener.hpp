#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eeros/control/ros2/EerosRosTools.hpp>
#include <vector>
#include <mutex>

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class generates the transformation between two frames.
 * It is recommended to create only one object of this class.
 * 
 * @tparam N defines how many tf generated from the tf tree
 * @tparam N Must fullfill: N == frames.size
 * 
 * It generates the transformation as follow:
 * out[i] => transformation between frames[i].first to frames[i].second
 */
template<uint8_t N>
class Ros2TransformListener: public Block {
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
   */
  Ros2TransformListener(rclcpp::Node::SharedPtr node,
                        const std::vector<std::pair<std::string, std::string>>& frames_, 
                        uint32_t queueSize=1000, 
                        tf2::Duration bufferCacheTime = tf2::BUFFER_CORE_DEFAULT_CACHE_TIME)
    : running(false),
      log(logger::Logger::getLogger('t')),
      frames(frames_) {
    static uint8_t objectCount = 0;
    static std::mutex mtx;
    if(frames.size() != N){
        std::ostringstream oss;
        oss << __FILE__ << "::" << __LINE__ << "\n frames (size: " << frames.size() << ") must have the same length as template param N=" << N;
        throw eeros::Fault(oss.str());
    }
    for(int i = 0; i<N; i++) {
      lastTime[i] = 0;
    }

    /* Checker to warn if more than one instance is created */
    mtx.lock();
    if(objectCount != 0){
      log.warn() << "Created multiple instance of class Ros2TransformListener.";
      log.warn() << " -> It is recommended to create only one instance!!!";
    }
    objectCount++;
    mtx.unlock();

    CHECK_RCLCPP_OK();
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock(), bufferCacheTime);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    log.info() << "Ros tf broadcaster Listener generated on node: " << node->get_name();
    running = true;
}

  virtual void run() {
    if(running) {
      for(uint8_t i = 0; i<N;i++) {
        try {
          msg = tf_buffer->lookupTransform(frames[i].second, frames[i].first, tf2::TimePointZero);
          qt.setX(msg.transform.rotation.x);
          qt.setY(msg.transform.rotation.y);
          qt.setZ(msg.transform.rotation.z);
          qt.setW(msg.transform.rotation.w);
          qt.normalize();
          tf2::Matrix3x3 tfmatrix(qt);
          tfmatrix.getRPY(poseRotation[0],
                          poseRotation[1],
                          poseRotation[2]);
          this->outPoseRotation[i].getSignal().setValue(poseRotation);
          this->outPoseRotation[i].getSignal().setTimestamp(System::getTimeNs());
          poseTranslation[0] = msg.transform.translation.x;
          poseTranslation[1] = msg.transform.translation.y;
          poseTranslation[2] = msg.transform.translation.z;
          this->outPoseTranslation[i].getSignal().setValue(poseTranslation);
          this->outPoseTranslation[i].getSignal().setTimestamp(System::getTimeNs());          
        } catch (const tf2::TransformException &e) {
          if((lastTime[i] + 2000000000) < eeros::System::getTimeNs()){
            log.warn() << "Couldn't process tf transformation for Frames:'" << frames[i].first <<"' to '" << frames[i].second << "'";
            log.warn() << "what(): " << e.what();
            lastTime[i] = eeros::System::getTimeNs();
          }
        }
      }
    }
  }

  /**
   * Get the output for rotation
   * 
   * @param index Index has the same function as getOutPoseRotation. It belongs to a tf
   * @return the Input
   * 
   * @throw eeros::control::IndexOutOfBoundsFault if index >= N
   */
  virtual control::Output<math::Vector3>& getOutPoseTranslation(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return outPoseTranslation[index];
  }

  /**
   * Get the output for rotation
   * 
   * @param index Index has the same function as getOutPoseTranslation. It belongs to a tf
   * @return the Input
   * 
   * @throw eeros::control::IndexOutOfBoundsFault if index >= N
   */
  virtual control::Output<math::Vector3>& getOutPoseRotation(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'"); 
    return outPoseRotation[index];
  }

 protected:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  bool running;
  eeros::logger::Logger log;
  const std::vector<std::pair<std::string, std::string>> frames;
  geometry_msgs::msg::TransformStamped msg;

  uint64_t lastTime[N];

  math::Vector3 poseTranslation;
  math::Vector3 poseRotation;
  tf2::Quaternion qt;
  control::Output<math::Vector3> outPoseTranslation[N];
  control::Output<math::Vector3> outPoseRotation[N];
};

} /* END Namespace: control */
} /* END Namespace: eeros */