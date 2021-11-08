#ifndef ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP
#define ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/RPLidar.hpp> 

#define LASER_COUNT_MAX 380

using namespace eeros::math;

namespace eeros {
namespace control {

class RPLidarInput : public Blockio<0,2,Matrix<LASER_COUNT_MAX,1>> {        
 public:
  /**
   * Constructs an rplidar input block, whic outputs data received from a RPLidar thread \n
   *
   * @param dev - string with device name
   * @param priority - execution priority of RPLidar thread to get sensors data
   */
  RPLidarInput(std::string dev, int priority = 5) : rplidar(dev, priority) { }
        
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RPLidarInput(const RPLidarInput& s) = delete; 
  
  /**
   * Runs the reading algorithm.
   *
   * output[0] = angles
   * output[1] = ranges
   * output timestamp = system time
   */
  virtual void run() {
    this->out[0].getSignal().setValue(rplidar.getAngles());
    this->out[1].getSignal().setValue(rplidar.getRanges());
    uint64_t ts = eeros::System::getTimeNs();
    this->out[0].getSignal().setTimestamp(ts);
  }
    
 private:                     
  eeros::hal::RPLidar rplidar;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * lidar instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, RPLidarInput& lidar) {
  os << "Block RPLidar input: '" << lidar.getName(); 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP */
