#include <eeros/logger/Logger.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/filter/MovingAverageFilter.hpp>

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
  setVal(0),
//   moving_avg({1.0}),
  td("td", period, true) 
  {
	moving_avg.getIn().connect(setVal.getOut());
	moving_avg.enable();
	
	td.addBlock(setVal);
	td.addBlock(moving_avg);
  }
  double test = {1.0, 2.0, 3.0};
  Constant<double> setVal;
  MovingAverageFilter<3,double,double> moving_avg(test);
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
  log.info() << "Pathplanner constant acceleration started...";
  
  ControlSystem cs;
  Periodic p1("p1", period, cs.td);
  PPSafetyProperties sp;
  SafetySystem ss(sp, period);
    
  // create periodic function for logging
  Lambda l1 ([&] () { });
  Periodic p2("p2", period, l1);
  p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    static int count = 0;
    log.info() << count << " -> " 
			   << cs.setVal.getOut().getSignal().getValue() << "  " ;
//                << cs.moving_avg.getOut().getSignal().getValue();
    if (count == 0) {
		cs.setVal.setValue(0.0);
    }
    if (count == 50) {
		cs.setVal.setValue(1.0);
    }
    count++;
  });
  
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  p1.after.push_back(p2); // make sure that logging happens after running of path planner
  executor.add(p1);
  executor.run();

  log.info() << "Test finished...";
}

