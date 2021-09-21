#include <eeros/logger/Logger.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/WrapAround.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::math;

double period = 0.1;

class ControlSystem {
 public:
  ControlSystem() : 
  setVal(0), //{0, 0}), 
  test_block(-3.1415, 3.1415),
  td("td", period, true)  
  {
	test_block.getIn().connect(setVal.getOut());
	
	td.addBlock(setVal);
	td.addBlock(test_block);
  }
  Constant<double> setVal;
  WrapAround<double> test_block;
//   Constant<eeros::math::Vector2> setVal;
//   WrapAround<eeros::math::Vector2> test_block;
  
  TimeDomain td;
};

class PPSafetyProperties : public SafetyProperties {
 public:
  PPSafetyProperties() : slState1("state 1") {
    addLevel(slState1);
    setEntryLevel(slState1);	
  };
  SafetyLevel slState1;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Wrap Around test started...";
  
  ControlSystem cs;
  Periodic p1("p1", period, cs.td);
  PPSafetyProperties sp;
  SafetySystem ss(sp, period);
    
  // create periodic function for logging
  Lambda l1 ([&] () { });
  Periodic p2("p2", period, l1);
  p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
	cs.setVal.setValue(cs.setVal.getValue() + 0.2);
    
//     eeros::math::Vector2 actVal = cs.setVal.getValue();
// 	eeros::math::Vector2 incr = {0.2, 0.2};
// 	cs.setVal.setValue(actVal + incr);
	
    log.info() << cs.setVal.getOut().getSignal().getValue() << "  "
               << cs.test_block.getOut().getSignal().getValue();
  });
  
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  p1.after.push_back(p2); // make sure that logging happens after running of path planner
  executor.add(p1);
  executor.run();

  log.info() << "Test finished...";
}

