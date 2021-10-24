#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <eeros/logger/Logger.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/sensor/BaumerOM70Input.hpp>

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
  ControlSystem() 
      : distSens1("192.168.0.252", 502, 1, 5),
        distSens2("192.168.0.253", 502, 1, 5),
        distSens3("192.168.0.254", 502, 1, 5),
        td("td", period, true) {
    td.addBlock(distSens1);
    td.addBlock(distSens2);
    td.addBlock(distSens3);
  }
  BaumerOM70Input distSens1, distSens2, distSens3;
  TimeDomain td;
};

class BaumerTestSafetyProperties : public SafetyProperties {
 public:
  BaumerTestSafetyProperties() : slState1("state 1") {
    addLevel(slState1);
    setEntryLevel(slState1);	
  };
  SafetyLevel slState1;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Program started - Read Baumer sensors data";
  
  ControlSystem cs;
  Periodic p1("p1", period, cs.td);
  BaumerTestSafetyProperties sp;
  SafetySystem ss(sp, period);
  
  Lambda l1 ([&] () { });
  Periodic p2("p2", period, l1);
  p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
      log.info() << cs.distSens1.getOut().getSignal().getValue() << ", " << 
      cs.distSens2.getOut().getSignal().getValue() << ", " << 
      cs.distSens3.getOut().getSignal().getValue() ;
  });
  
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  p1.after.push_back(p2); // make sure that logging happens after running of control system
  executor.add(p1);
  executor.run();
          
  log.info() << "Program finished...";
  return 0;
}

