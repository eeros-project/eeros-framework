#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/KeyboardInput.hpp>
#include <eeros/task/Lambda.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::safety;

double period = 0.01;

class ControlSystem {
 public:
  ControlSystem() : keyboard(), td("td", period, true) {
    keyboard.setName("keyboard");
    keyboard.getOut().getSignal().setName("position");
    keyboard.getEsc().getSignal().setName("esc event");
    td.addBlock(keyboard);
    Executor::instance().add(td);
  }
  KeyboardInput keyboard;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties() : slFirst("first level"), slSecond("second level"), seGoUp("go to second level"), seGoDown("go to first level") {
    Input<bool>* in1 = HAL::instance().getLogicInput("escKeyboardButton", false);
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

int main() {
  signal(SIGINT, signalHandler);
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Keyboard test started ...";
  
  ControlSystem cs;
  TestSafetyProperties sp;
  SafetySystem ss(sp, period);
    
  Lambda l1 ([&] () { });
  Periodic periodic("per1", 1, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.keyboard.getEsc().getSignal();
    log.info() << cs.keyboard.getOut().getSignal();
  });
    
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();

  log.info() << "Test finished...";
}
