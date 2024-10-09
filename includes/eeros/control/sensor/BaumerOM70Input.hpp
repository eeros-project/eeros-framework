#ifndef ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP
#define ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP

#ifdef EEROS_USE_MODBUS

#include <string>
#include <thread>
#include <eeros/control/Blockio.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/BaumerOM70.hpp>

using namespace eeros::math;
using namespace eeros::hal;
using namespace eeros::logger;

namespace eeros {
namespace control {

/**
 * This block reads a Baumer OM70 distance sensor over MODBUS and outputs the distance.
 *
 * @since v1.3
 */

class BaumerOM70Input: public Blockio<0,1,double> {
 public:
  /**
   * Constructs an input block to get data from baumer OM70 sensor. Output is a laser distance \n
   * Calls BaumerOM70Input(std::string dev, int port, int slave, int priority)
   *
   * @see  BaumerOM70Input(std::string dev, int port, int slave, int priority)
   * @param dev - string with device name
   * @param port - port for sensor data read (modbus interface)
   * @param slaveId - sensor slave number (modbus interface)
   * @param priority - execution priority or BaumerOM70 thread, to get sensors data
   */
  BaumerOM70Input(std::string dev, int port, int slaveId, int priority = 5) 
      : om70(dev, port, slaveId, priority), log(Logger::getLogger()) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  BaumerOM70Input(const BaumerOM70Input& s) = delete; 
  
  /**
   * Gets input data from Baumer OM70 thread and outputs them
   */
  virtual void run() {
    this->out.getSignal().setValue(om70.getDistance());
    this->out.getSignal().setTimestamp(eeros::System::getTimeNs());
  }
      
 protected:        
  BaumerOM70 om70;
  Logger log;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * sensor instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, BaumerOM70Input& sensor) {
  os << "Block BaumerOM70 input: '" << sensor.getName(); 
  return os;
}

}
}

#endif
#endif /* ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP */


