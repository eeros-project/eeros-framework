#ifndef ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_
#define ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/SBGEllipseA.hpp>

using namespace eeros::hal;
using namespace eeros::math;
using namespace eeros::logger;

namespace eeros {
namespace control {

/**
 * This block reads a SBG Ellipse-A IMU sensor over USB and outputs the distance.
 *
 * @since v1.3
 */

class SBGEllipseAInput: public Blockio<0,0> {
 public:
  /**
   * Constructs an input block to get data from SBG Elliipse-A IMU sensor. \n
   *
   * @param dev - string with device name (USB)
   * @param priority - execution priority or SBGEllipseA thread, to get sensors data
   */
  SBGEllipseAInput(std::string dev, int priority = 5) : sbg(dev, priority), log(Logger::getLogger()) { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  SBGEllipseAInput(const SBGEllipseAInput& s) = delete; 
  
  /**
   * Gets input data from SBGEllipseA thread and outputs them
   */
  virtual void run() {
    euler.getSignal().setValue(SBGEllipseA::eulerData);
    quaternion.getSignal().setValue(SBGEllipseA::quatData);
    acc.getSignal().setValue(SBGEllipseA::accData);
    gyro.getSignal().setValue(SBGEllipseA::gyroData);
    timestamp.getSignal().setValue(SBGEllipseA::timestampEuler);
      
    // Timestamps
    uint64_t ts = eeros::System::getTimeNs();
    euler.getSignal().setTimestamp(ts);
    quaternion.getSignal().setTimestamp(ts);
    acc.getSignal().setTimestamp(ts);
    gyro.getSignal().setTimestamp(ts);
    timestamp.getSignal().setTimestamp(ts);
  }
  
  /**
   * Gets the output orientation, expressed in euler angles 
   * 
   * @return euler output
   */
  virtual Output<Vector3>& getOutEulerAngle() {
    return euler;
  }
  
  /**
   * Gets the output orientation, expressed in quaternions 
   * 
   * @return quaternion output
   */
  virtual Output<Vector4>& getOutQuaternion(){
    return quaternion;
  }
  
  /**
   * Gets the output linear accelerations ax, ay, az from accelerometer 
   * 
   * @return acceleration output
   */
  virtual Output<Vector3>& getOutAcc() {
    return acc;
  }
  
  /**
   * Gets the output angular velocities wx, wy, wz from gyro 
   * 
   * @return gyro output
   */
  virtual Output<Vector3>& getOutGyro() {
    return gyro;
  }
  
  /**
   * Gets the output timestamp of the sensor signal 
   * 
   * @return timestamp output
   */
  virtual Output<uint32_t>& getOutTimestamp() {
    return timestamp;
  }
  
 protected:
  Output<Vector3> euler, acc, gyro;
  Output<Vector4> quaternion;
  Output<uint32_t> timestamp;
  
  SBGEllipseA sbg;
  Logger log;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * imu instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, SBGEllipseAInput& imu) {
  os << "Block SBGEllipseAInput input: '" << imu.getName(); 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_ */

