#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/SpaceNavigatorInput.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::safety;

double period = 0.01;

class ControlSystem {
 public:
  ControlSystem() : sn("/dev/input/event18"), td("td1", period, true) {
    sn.setName("space navigator");
    sn.getOut().getSignal().setName("space nav position signal");
    sn.getRotOut().getSignal().setName("space nav rotation signal");
    sn.getButtonOut().getSignal().setName("space nav button signal");
    td.addBlock(sn);
    Executor::instance().add(td);
  }
  SpaceNavigatorInput sn;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
public:
  TestSafetyProperties() : slFirst("first level"), slSecond("second level"), seGoUp("go to second level"), seGoDown("go to first level") {
    auto* in1 = HAL::instance().getLogicInput("SpaceNavButtonL", false);
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

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();

  log.info() << "Space Navigator test started...";
  
  ControlSystem cs;
  TestSafetyProperties sp;
  SafetySystem ss(sp, period);
  
  Lambda l1 ([&] () { });
  Periodic periodic("per1", 1, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.sn.getOut().getSignal();
    log.info() << cs.sn.getRotOut().getSignal();
    log.info() << cs.sn.getButtonOut().getSignal();
  });
    
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();

  log.info() << "Test finished...";
}
