#ifndef ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP

#include <cmath>
#include <mutex>
#include <eeros/control/Block1i.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/NaNOutputFault.hpp>

namespace eeros {
namespace control {

template < typename T = double >
class PeripheralOutput : public Block1i<T> {
 public:
  PeripheralOutput(std::string id, bool exclusive = true) : hal(hal::HAL::instance()) {
    systemOutput = dynamic_cast<hal::Output<T>*>(hal.getOutput(id, exclusive));
    if(systemOutput == nullptr) throw Fault("Peripheral output '" + id + "' not found!");
  }
            
  virtual void run() {
    std::lock_guard<std::mutex> lock(mtx);
    val = this->in.getSignal().getValue();
    auto isSafe = false;
    if(std::isnan(val) || std::isinf(val)) {
      val = systemOutput->safe;
      isSafe = true;
    }
    systemOutput->set(val);
    systemOutput->setTimestampSignalIn(this->in.getSignal().getTimestamp());
    if (isSafe) throw NaNOutputFault("NaN written to output '" + 
                                     this->getName() + "', set to safe level is safe level is defined");
  }
            
  virtual T getValue () {
    std::lock_guard<std::mutex> lock(mtx);
    return val;
  }
            
  template<typename ... ArgTypesOut>
  void callOutputFeature(std::string featureName, ArgTypesOut... args){
    hal.callOutputFeature(systemOutput, featureName, args...);
  }
            
 private:
  hal::HAL& hal;
  hal::Output<T>* systemOutput;
  T val;
  std::mutex mtx;
};

};
};

#endif // ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
