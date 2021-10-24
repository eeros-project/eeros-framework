#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <eeros/logger/Logger.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/sensor/PixyCamInput.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::task;

double period = 0.01;

void signalHandler(int signum) {
  SafetySystem::exitHandler();
}

class ControlSystem {
 public:
  ControlSystem() : cam("/dev/input/event18"), td("td", period, true) {
    td.addBlock(cam);
  }
  PixyCamInput cam;
  TimeDomain td;
};

class PixyTextSafetyProperties : public SafetyProperties {
 public:
  PixyTextSafetyProperties() : slState1("state 1") {
    addLevel(slState1);
    setEntryLevel(slState1);	
  };
  SafetyLevel slState1;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Program started - Read out PixyCam sensors data";
  
  ControlSystem cs;
  Periodic p1("p1", period, cs.td);
  PixyTextSafetyProperties sp;
  SafetySystem ss(sp, period);
  
  Lambda l1 ([&] () { });
  Periodic p2("p2", period, l1);
  p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    log.info() << cs.cam.getOut().getSignal().getValue();
  });
  
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  p1.after.push_back(p2); // make sure that logging happens after the control system has run
  executor.add(p1);
  executor.run();
      
  log.info() << "Program finished...";
  return 0;
}


