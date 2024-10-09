#ifndef ORG_EEROS_HAL_RPLIDAR_HPP_
#define ORG_EEROS_HAL_RPLIDAR_HPP_

#ifdef EEROS_USE_RPLIDAR

#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <rplidar.h>
#include <atomic>

#define LASER_COUNT_MAX 380

using namespace eeros::math;
using namespace eeros::logger;
using namespace rp::standalone::rplidar;

namespace eeros {
namespace hal {

/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::RPLidarInput class. 
 * Do not use it directly.
 *
 * @since v1.3
 */
class RPLidar : public eeros::Thread {
 public:
  /**
   * Constructs a Thread to get RPLidar (Laserscanner) sensors data. \n
   *
   * @param dev - string with device name (USB)
   * @param priority - execution priority of thread to get sensors data
   */
  explicit RPLidar(std::string dev, int priority);
  
  /**
   * Destructs a Thread to get Baumer OM70 sensors data \n
   */
  ~RPLidar();

  /**
   * Gets scanning frequency
   * 
   * @return frequency
   */
  float getScanFrequency();
  
  /**
   * Gets angles where a range has been measured
   * 
   * @return laser angles
   */
  Vector<LASER_COUNT_MAX,double> getAngles();
  
  /**
   * Gets range measurements
   * 
   * @return laser ranges
   */
  Vector<LASER_COUNT_MAX,double> getRanges();
  
  /**
   * Gets range intensities
   * 
   * @return laser intensities
   */
  Vector<LASER_COUNT_MAX,double> getIntensities();
  
 private:
  virtual void run();
  
  int bufSize;
  RPlidarDriver* ld;
  std::atomic<bool> starting;
  std::atomic<bool> running;
  Logger log;
  Vector<LASER_COUNT_MAX,double> angles, ranges, intensities; 
};

}
}

#endif
#endif /* ORG_EEROS_HAL_RPLIDAR_HPP_ */
