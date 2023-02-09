#ifndef ORG_EEROS_CORE_SYSTEM_HPP_
#define ORG_EEROS_CORE_SYSTEM_HPP_

#include <stdint.h>

namespace eeros {

static bool rosTimeIsUsed __attribute__((unused)) = false;

/**
 * This is the base class for accessing system time.
 * The class may be used solely in a static way.
 *
 * @since v0.4
 */
class System {
 public:
  /**
   * Returns the clock resolution in seconds
   *
   * @return clock resolution in sec
   */
  static double getClockResolution();

  /**
   * Returns the system time in seconds.
   *
   * @return system time in sec
   */
  static double getTime();

  /**
   * Returns the system time in nanoseconds.
   *
   * @return system time in nsec
   */
  static uint64_t getTimeNs();

#if defined (USE_ROS) || defined (USE_ROS2)
  /**
   * Makes the system reading the system time from ROS.
   */
  static void useRosTime();
#endif

 private:
  System();
};

}

#endif /* ORG_EEROS_CORE_SYSTEM_HPP_ */
