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
        aIn("analogIn"),
        dIn("digitalIn"),
        aOut("analogOut"),
        dOut("digitalOut"),
        timedomain("Main time domain", ts, true) {
    aIn.getOut().getSignal().setName("analog in");
    dIn.getOut().getSignal().setName("digital in");
    aOut.getIn().connect(c1.getOut());
    dOut.getIn().connect(dIn.getOut());
    timedomain.addBlock(c1);
    timedomain.addBlock(aOut);
    timedomain.addBlock(aIn);
    timedomain.addBlock(dIn);
    timedomain.addBlock(dOut);
    
    eeros::Executor::instance().add(timedomain);
  }

  Constant<> c1;
  PeripheralInput<double> aIn;
  PeripheralInput<bool> dIn;
  PeripheralOutput<double> aOut;
  PeripheralOutput<bool> dOut;
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
        log.info() << cs.aIn.getOut().getSignal();
        log.info() << cs.dIn.getOut().getSignal();
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
  log.show(LogLevel::TRACE);
  log.info() << "ROS Test 2 started";

  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);

  ControlSystem cs(dt);
  ROSTestSafetyProperties sp(cs);
  SafetySystem ss(sp, dt);
  
  signal(SIGINT, signalHandler);	
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  executor.run();

  log.info() << "ROS Test2 end";	
  return 0;
}
