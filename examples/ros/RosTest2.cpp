#include <eeros/hal/HAL.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/core/Executor.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::control;

class ControlSystem {
 public:
  ControlSystem(double ts)
      : c1(0.5),
        doubleIn("doubleIn"),
        boolIn("boolIn"),
        doubleOut("doubleOut"),
        boolOut("boolOut"),
        timedomain("Main time domain", ts, true) {
    doubleOut.getIn().connect(c1.getOut());
    boolOut.getIn().connect(boolIn.getOut());
    timedomain.addBlock(c1);
    timedomain.addBlock(doubleOut);
    timedomain.addBlock(doubleIn);
    timedomain.addBlock(boolIn);
    timedomain.addBlock(boolOut);
    
    eeros::Executor::instance().add(timedomain);
  }

  Constant<> c1;
  PeripheralInput<double> doubleIn;
  PeripheralInput<bool> boolIn;
  PeripheralOutput<double> doubleOut;
  PeripheralOutput<bool> boolOut;
  TimeDomain timedomain;
};

class ROSTestSafetyProperties : public SafetyProperties {
public:
  ROSTestSafetyProperties(ControlSystem& cs) : slOff("off"), cs(cs), log(Logger::getLogger()) {
    addLevel(slOff);
    setEntryLevel(slOff);
    slOff.setLevelAction([&](SafetyContext* privateContext) {
      if ((slOff.getNofActivations() % 5) == 0) {
        cs.c1.setValue(cs.c1.getOut().getSignal().getValue() + 0.01);
        log.info() << cs.doubleIn.getOut().getSignal() << "   " << cs.boolIn.getOut().getSignal();
      }
    });
  }
  
  SafetyLevel slOff;
  ControlSystem& cs;
  Logger log;
};

void signalHandler(int signum) {
  Executor::stop();
}

int main(int argc, char **argv) {	
  double dt = 0.2;
  
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "ROS Test 2 started";

  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);

  ControlSystem controlSystem(dt);
  ROSTestSafetyProperties safetyProperties(controlSystem);
  eeros::safety::SafetySystem safetySystem(safetyProperties, dt);

  signal(SIGINT, signalHandler);
  auto& executor = Executor::instance();
  executor.setMainTask(safetySystem);
  executor.run();

  log.info() << "ROS Test2 end";
  return 0;
}
