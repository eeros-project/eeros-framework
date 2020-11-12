#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/SignalChecker.hpp>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;

class ControlSystem {
 public:
  ControlSystem() : c(0.5), checker(0, 5) {
    i.getOut().getSignal().setName("integrator output");
    i.getIn().connect(c.getOut());
    checker.setName("check integrator level");
    checker.getIn().connect(i.getOut());
  }

  Constant<> c;
  I<> i;
  SignalChecker<> checker;
};

class SafetyPropertiesTest : public SafetyProperties {
 public:
  SafetyPropertiesTest(ControlSystem& cs, double ts) 
      : cs(cs),
        seStartRampingUp("start ramping up"),
        seReset("reset"),
        slStart("start"),
        slRampingUp("ramping up") {   
    
    // ############ Add levels ############
    addLevel(slStart);
    addLevel(slRampingUp);
    
    // ############ Add events to the levels ############
    slStart.addEvent(seStartRampingUp, slRampingUp, kPrivateEvent);
    slRampingUp.addEvent(seReset, slStart, kPublicEvent);

    // Define and add level functions
    slStart.setLevelAction([&,ts](SafetyContext* privateContext) {
      cs.i.disable();
      cs.i.setInitCondition(0);
      if(slStart.getNofActivations() * ts > 3) {
        privateContext->triggerEvent(seStartRampingUp);
        cs.checker.reset();
        cs.i.enable();
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
  
  log.info() << "Safety System Example 2 started...";
  
  // Create and initialize safety system
  double period = 0.01;
  ControlSystem cs;
  SafetyPropertiesTest sp(cs, period);
  SafetySystem safetySys(sp, period);
  cs.checker.registerSafetyEvent(safetySys, sp.seReset);
  cs.checker.setActiveLevel(sp.slRampingUp);
  
  TimeDomain td("td1", 0.5, true);
  Periodic periodic("per1", 0.5, td);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    log.info() << cs.i.getOut().getSignal();
  });
  
  td.addBlock(cs.c);
  td.addBlock(cs.i);
  td.addBlock(cs.checker);
  
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(safetySys);
  executor.add(periodic);
  executor.run();
  
  log.info() << "Test finished...";
}
