#ifndef ORG_EEROS_HAL_BAUMEROM70_HPP_
#define ORG_EEROS_HAL_BAUMEROM70_HPP_

#ifdef EEROS_USE_MODBUS

#include <eeros/core/Thread.hpp>
#include <eeros/logger/Logger.hpp>
#include <modbus/modbus.h>    
#include <atomic>

using namespace eeros::logger;

namespace eeros {
namespace hal {
    
/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::BaumerOM70Input class. 
 * Do not use it directly.
 * 
 * @since v1.3
 */
class BaumerOM70 : public eeros::Thread {
 public: 
  /**
   * Constructs a thread to get Baumer OM70 sensors data \n
   * Calls BaumerOM70(std::string dev, int port, int slave_id, int priority)
   *
   * @see BaumerOM70(std::string dev, int port, int slave_id, int priority)
   * @param dev - string with device name
   * @param port - port for sensor data read (modbus interface)
   * @param slaveId - sensor slave number (modbus interface)
   * @param priority - execution priority or BaumerOM70 thread, to get sensors data
   */
  explicit BaumerOM70(std::string dev, int port, int slaveId, int priority);
    
  /**
   * Destructor
   */
  ~BaumerOM70();
    
  /**
   * Gets data from Baumer OM70 sensor 
   * Is called by the run() method
   * @see run()
   */
  void getMeasurements();
    
  /**
   * Gets range of measurement limits set 
   */
  void getMeasRangeLimits();
    
  /**
   * Gets IP-Adress of Baumer OM70 sensor 
   */
  void getIpAddress();
    
  /**
   * Switches ON Laser of Baumer OM70 sensor 
   */
  void switchOnLaser();

  /**
   * Switches OFF Laser of Baumer OM70 sensor 
   */
  void switchOffLaser();
    
  /**
   * Gets distance measurement
   * 
   * @return distance
   */
  float getDistance();

 private:
  /**
   * Runs methods for data acquisition from sensor
   * Calls getMeasurements()
   */
  virtual void run();
    
  std::atomic<bool> starting;
  std::atomic<bool> running;
  int id;
  modbus_t *ctx;
  uint16_t tabReg[32];
  float distance;
  uint32_t timestampUs;
  float measRate;
  Logger log;
};

}
}
#endif
#endif /* ORG_EEROS_HAL_BAUMEROM70_HPP_ */


