#ifndef ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP
#define ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP

#ifdef EEROS_USE_REALSENSE

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/RealsenseT265.hpp>

using namespace eeros::math;

namespace eeros {
namespace control {

/**
 * This block reads a Realsense T256 camera over USB3.
 *
 * @since v1.3
 */
  
class RealsenseT265Input : public Blockio<0,0> {
 public:
  /**
   * Constructs an input block to get data from Realsense Tracking T265 sensor. \n
   * Calls RealsenseT265Input(std::string dev, int priority)
   *
   * @see  RealsenseT265Input(std::string dev, int priority)
   * @param dev - string with device name (USB 3.0)
   * @param priority - execution priority of thread to get sensors data
   */
  RealsenseT265Input(std::string dev, int priority = 5) : t265(dev, priority) {}
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RealsenseT265Input(const RealsenseT265Input& s) = delete; 
  
  
  /**
   * Gets input data from Realsense Tracking T265 Thread and outputs them
   */
  virtual void run() {
    translation.getSignal().setValue(t265.translation);
    velocity.getSignal().setValue(t265.velocity);
    acceleration.getSignal().setValue(t265.acceleration);
    angularVelocity.getSignal().setValue(t265.angVelocity);
    angularAcceleration.getSignal().setValue(t265.angAcceleration);
    quaternion.getSignal().setValue(t265.quaternion);
      
    // Timestamps
    uint64_t ts = eeros::System::getTimeNs();
    translation.getSignal().setTimestamp(ts);
    velocity.getSignal().setTimestamp(ts);
    acceleration.getSignal().setTimestamp(ts);
    angularVelocity.getSignal().setTimestamp(ts);
    angularAcceleration.getSignal().setTimestamp(ts);
    quaternion.getSignal().setTimestamp(ts);
  }
  
  /**
   * Gets the output translation x, y, z of tracking system, referred to position at data acquisition start
   * 
   * @return translation
   */
  virtual Output<Vector3>& getTranslation(){
    return translation;
  }
  
  /**
   * Gets the output linear velocity vx, vy, vz  of tracking system
   * 
   * @return velocity
   */
  virtual Output<Vector3>& getVelocity(){
    return velocity;
  }
    
  /**
   * Gets the output linear acceleration ax, ay, az of tracking system
   * 
   * @return acceleration
   */
  virtual Output<Vector3>& getAcceleration(){
    return acceleration;
  }
      
  /**
   * Gets the output angular velocity wx, wy, wz  of tracking system
   * 
   * @return angularVelocity
   */
  virtual Output<Vector3>& getAngularVelocity(){
    return angularVelocity;
  }
      
  /**
   * Gets the output angular acceleration alx, aly, alz of tracking system 
   * 
   * @return angularAcceleration
   */
  virtual Output<Vector3>& getAngularAcceleration(){
    return angularAcceleration;
  }
  
  /**
   * Gets the output orientation of the tracking system, expressed in quaternions
   * 
   * @return quaternion
   */
  virtual Output<Vector4>& getQuaternion(){
    return quaternion;
  }
  
 private:
  Output<Vector3> translation, velocity, acceleration, angularVelocity, angularAcceleration;
  Output<Vector4> quaternion;
  eeros::hal::RealsenseT265 t265;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * sensor instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, RealsenseT265Input& sensor) {
  os << "Block RealsenseT256 input: '" << sensor.getName(); 
  return os;
}

}
}

#endif
#endif /* ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP */

