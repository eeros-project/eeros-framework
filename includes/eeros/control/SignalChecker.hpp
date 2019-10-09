#ifndef ORG_EEROS_SIGNAL_CHECKER_HPP_
#define ORG_EEROS_SIGNAL_CHECKER_HPP_

#include <eeros/control/Block1i.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>


namespace eeros {
namespace control {

using namespace safety;

template<typename Tsig = double, typename Tlim = Tsig, bool checkNorm = false>
class SignalChecker : public Block1i<Tsig> {
 public:
  SignalChecker(Tlim lowerLimit, Tlim upperLimit) :
      lowerLimit(lowerLimit),
      upperLimit(upperLimit),
      fired(false),
      safetySystem(nullptr),
      safetyEvent(nullptr),
      activeLevel(nullptr) {}


  virtual void run() override {
    auto val = this->in.getSignal().getValue();
    if (!fired) {
      if (limitsExceeded<bool>(val)) {
        if (safetySystem != nullptr && safetyEvent != nullptr) {
          if (activeLevel == nullptr ||
           (activeLevel != nullptr && safetySystem->getCurrentLevel() >= *activeLevel)
           ) {
            log.warn() << "Signal checker \'" + this->getName() + "\' fires!";
            safetySystem->triggerEvent(*safetyEvent);
            fired = true;
          }
        }
      }
    }
  }


  virtual void setLimits(Tlim lowerLimit, Tlim upperLimit) {
    this->lowerLimit = lowerLimit;
    this->upperLimit = upperLimit;
  }


  virtual void reset() {
    fired = false;
  }


  virtual void registerSafetyEvent(SafetySystem &ss, SafetyEvent &e) {
    safetySystem = &ss;
    safetyEvent = &e;
  }


  virtual void setActiveLevel(SafetyLevel &level) {
    activeLevel = &level;
  }


 protected:
  Tlim lowerLimit, upperLimit;
  bool fired;
  SafetySystem *safetySystem;
  SafetyEvent *safetyEvent;
  SafetyLevel *activeLevel;
  eeros::logger::Logger log{};


 private:
  template<typename S>
  typename std::enable_if<!checkNorm, S>::type limitsExceeded(Tsig value) {
    return !(value > lowerLimit && value < upperLimit);
  }


  template<typename S>
  typename std::enable_if<checkNorm, S>::type limitsExceeded(Tsig value) {
    return !(value.norm() > lowerLimit && value.norm() < upperLimit);
  }

};
}
}

#endif /* ORG_EEROS_SIGNAL_CHECKER_HPP_ */
