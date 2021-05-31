#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/MouseInput.hpp>
#include <eeros/task/Lambda.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::safety;

double period = 0.01;

class ControlSystem {
 public:
  ControlSystem() : mouse("/dev/input/event6", scale, min, max), td("td1", period, true) {
    mouse.setName("mouse");
    mouse.getOut().getSignal().setName("position");
    mouse.getButtonOut().getSignal().setName("events");
    td.addBlock(mouse);
    Executor::instance().add(td);
  }
  Vector4 scale{0.0001, 0.0001, 0.001, 0.1}, min{-0.1}, max{0.2};
  MouseInput mouse;
  TimeDomain td;
};

class MouseTestSafetyProperties : public SafetyProperties {
public:
  MouseTestSafetyProperties() : slFirst("first level"), slSecond("second level"), seGoUp("go to second level"), seGoDown("go to first level") {
    auto* in1 = HAL::instance().getLogicInput("middleMouseButton", false);
    criticalInputs = { in1 };

    addLevel(slFirst);
    addLevel(slSecond);
    
    slFirst.setInputActions({ check(in1, false, seGoUp) });
    slSecond.setInputActions({ check(in1, true, seGoDown) });
    
    slFirst.addEvent(seGoUp, slSecond, kPrivateEvent);
    slSecond.addEvent(seGoDown, slFirst, kPrivateEvent);

    setEntryLevel(slFirst);
  }
  SafetyLevel slFirst;
  SafetyLevel slSecond;
  SafetyEvent seGoUp;
  SafetyEvent seGoDown;
};

void signalHandler(int signum) {
  Executor::instance().stop();
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  
  log.info() << "Mouse test started ...";
  
  ControlSystem cs;
  MouseTestSafetyProperties sp;
  SafetySystem ss(sp, period);
    
  Lambda l1 ([&] () { });
  Periodic periodic("per1", 1, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.mouse.getOut().getSignal() << cs.mouse.getButtonOut().getSignal();
  });
    
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();

  log.info() << "Test finished...";
}
