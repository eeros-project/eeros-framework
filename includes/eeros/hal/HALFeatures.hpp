#ifndef ORG_EEROS_HALFEATURES_HAL_HPP_
#define ORG_EEROS_HALFEATURES_HAL_HPP_

#include <map>
#include <string>

namespace eeros {
namespace hal {

enum Direction {
  In,
  Out
};

enum Type {
  Logic,
  Real
};

const std::map<std::string, Direction> directionOfChannel = {
  { "DigIn",     In },
  { "DigOut",    Out },
  { "AnalogOut", Out },
  { "AnalogIn",  In },
  { "Pwm",       Out },
  { "Watchdog",  In },
  { "Fqd",       In }
};

const std::map<std::string, Type> typeOfChannel = {
  { "DigIn",     Logic },
  { "DigOut",    Logic },
  { "AnalogOut", Real },
  { "AnalogIn",  Real },
  { "Pwm",       Real },
  { "Watchdog",  Logic },
  { "Fqd",       Real }
};

}
}

#endif /* ORG_EEROS_HALFEATURES_HAL_HPP_ */
