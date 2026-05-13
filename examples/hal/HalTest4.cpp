#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/TimeDomain.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::task;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;

class ControlSystem {
 public:
  ControlSystem(double ts) : in("ch1"), out("ch0"), td("Main time domain", ts, true) {
    out.getIn().connect(in.getOut());
    td.addBlock(in);
    td.addBlock(out);
    Executor::instance().add(td);
  }

  PeripheralInput<bool> in;		// digital input
  PeripheralOutput<bool> out;		// digital output
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties()
      : slOne("slOne"),
        slTwo("slTwo"),
        seStart("seStart"),
        seStop("seStop") {
    hal::Input<bool>* in = HAL::instance().getLogicInput("ch16", false);
    criticalInputs = {in};
    slOne.setInputActions({check(in, false, seStart)});
    slTwo.setInputActions({check(in, true, seStop)});

    hal::Output<bool>* out = HAL::instance().getLogicOutput("ch3", false);
    criticalOutputs = {out};
    slOne.setOutputActions({set(out, false)});
    slTwo.setOutputActions({set(out, true)});

    slOne.addEvent(seStart, slTwo, kPrivateEvent);
    slTwo.addEvent(seStop, slOne, kPrivateEvent);

    addLevel(slOne);
    addLevel(slTwo);
    setEntryLevel(slOne);
  }

  SafetyLevel slOne, slTwo;
  SafetyEvent seStart, seStop;
};

int main(int argc, char **argv){
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
    
  log.info() << "HAL gpio test started...";
  
  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);

  const double dt = 0.01;
  ControlSystem cs(dt);
  TestSafetyProperties sp;
  SafetySystem ss(sp, dt);
 
  auto &executor = Executor::instance();
  executor.setMainTask(ss);
  executor.run();
    
  log.info() << "end...";
  return 0;
}
