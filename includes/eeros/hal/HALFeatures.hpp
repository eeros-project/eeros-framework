#ifndef ORG_EEROS_HALFEATURES_HAL_HPP_
#define ORG_EEROS_HALFEATURES_HAL_HPP_

#include <map>
#include <string>
#include <eeros/SIUnit.hpp>

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

const std::map<std::string, SIUnit> typeOfUnit = {
  { "W",    Watt },
  { "N",    Newton },
  { "J",    Joule },
  { "V",    Volt },
  { "rad",  Radian }
};

}
}

#endif /* ORG_EEROS_HALFEATURES_HAL_HPP_ */
