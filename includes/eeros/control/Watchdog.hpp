#ifndef ORG_EEROS_CONTROL_WDT_HPP_
#define ORG_EEROS_CONTROL_WDT_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/hal/HAL.hpp>

namespace eeros {
namespace control {

/**
 * A watchdog timer counts backwards and allows to stop critical outputs, e.g.
 * drive signals when a timeout occures. The timer must be periodically
 * reset to its start value. When timeout happened, timer must be rearmed.
 *
 * @since v1.4
 */
class Watchdog : public Block {
 public:
  /**
   * Constructs a watchdog instance.\n
   */
  Watchdog() : hal(eeros::hal::HAL::instance()) {
    wdt = hal.getLogicInput("Wdt", false);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Watchdog(const Watchdog& s) = delete;

  /**
   * Runs the watchdog block. Will reset the timer to its timeout value.
   */
  virtual void run() {
    hal.callInputFeature(wdt, "reset");
  }

  /**
   * Returns the current status of a watchdog timer.
   *
   * @return - true -> timer still running, false -> counter has reached zero
   */
  bool getStatus() {
    return wdt->get();
  }

  /**
   * Arms the watchdog timer. That means, that the counter is reloaded with
   * its start value and the rearm bit is set in order to start the timer.
   */
  void arm() {
    hal.callInputFeature(wdt, "arm");
  }

 protected:
  eeros::hal::HAL& hal;
  eeros::hal::Input<bool>* wdt;
};

/********** Print functions **********/
template <typename In1T = double, typename In2T = double, typename OutT = double>
std::ostream& operator<<(std::ostream& os, Watchdog& wdt) {
  os << "Block watchdog: '" << wdt.getName() << "'";
        return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_WDT_HPP_ */

