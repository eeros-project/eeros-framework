#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/SignalChecker.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

class ControlSystem {
 public:
  ControlSystem() : c(0.5), checker(-0.001, 5), td("td1", 0.5, true) {
    i.getOut().getSignal().setName("integrator output");
    i.getIn().connect(c.getOut());
    checker.setName("check integrator level");
    checker.getIn().connect(i.getOut());
    td.addBlock(c);
    td.addBlock(i);
    td.addBlock(checker);
    Executor::instance().add(td);
  }

  Constant<> c;
  I<> i;
  SignalChecker<> checker;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties(ControlSystem& cs, double ts) 
      : cs(cs),
        seStartRampingUp("start ramping up"),
        seReset("reset"),
        slStart("start"),
        slRampingUp("ramping up") {   
    
    // levels
    addLevel(slStart);
    addLevel(slRampingUp);
    
    // Add events to the levels
    slStart.addEvent(seStartRampingUp, slRampingUp, kPrivateEvent);
    slRampingUp.addEvent(seReset, slStart, kPublicEvent);

    // Define and add level functions
    slStart.setLevelAction([&,ts](SafetyContext* privateContext) {
      cs.i.setInitCondition(0);
      if(slStart.getNofActivations() * ts > 3) {
        privateContext->triggerEvent(seStartRampingUp);
        cs.checker.reset();
      }
    });

    // Define entry level
    setEntryLevel(slStart); 
  }

  ControlSystem& cs;
  SafetyEvent seStartRampingUp;
  SafetyEvent seReset;
  SafetyLevel slStart;
  SafetyLevel slRampingUp;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  
  log.info() << "System test 3 started...";
  
  double period = 0.01;
  ControlSystem cs;
  TestSafetyProperties sp(cs, period);
  SafetySystem ss(sp, period);
  cs.checker.registerSafetyEvent(ss, sp.seReset);
  cs.checker.setActiveLevel(sp.slRampingUp);
  cs.i.setActiveLevel(ss, sp.slRampingUp);
  
  Lambda l2 ([&] () { });
  Periodic periodic("p2", 0.5, l2);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    log.info() << cs.i.getOut().getSignal();
  });
  
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();
  
  log.info() << "Test finished...";
}
