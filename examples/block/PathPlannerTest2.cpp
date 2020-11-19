#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PathPlannerConstJerk.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/task/Lambda.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::math;

double period = 0.1;

class ControlSystem {
 public:
  ControlSystem() : pp(50.0, 1.0, period), td("td", period, true) {
    pp.setName("ppcj");
    pp.getPosOut().getSignal().setName("pp pos out");
    pp.getVelOut().getSignal().setName("pp vel out");
    pp.getAccOut().getSignal().setName("pp acc out");
    pp.getJerkOut().getSignal().setName("pp jerk out");
    td.addBlock(pp);
  }
  PathPlannerConstJerk<Matrix<2,1,double>> pp;
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
  Logger log = Logger::getLogger('M');
  log.info() << "Pathplanner constant jerk started...";
  
  ControlSystem cs;
  Periodic p1("p1", period, cs.td);
  PPSafetyProperties sp;
  SafetySystem ss(sp, period);
    
  // create periodic function for logging
  Lambda l1 ([&] () { });
  Periodic p2("p2", period, l1);
  p2.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    static int count = 0;
    log.info() << cs.pp.getJerkOut().getSignal().getValue() << "  " 
               << cs.pp.getAccOut().getSignal().getValue() << "  " 
               << cs.pp.getVelOut().getSignal().getValue() << "  " 
               << cs.pp.getPosOut().getSignal().getValue();
    if (count == 1) {
      Matrix<2,1,double> start{0, 0}, end{10, 20};
      log.warn() << "start trajectory from " << start << " to " << end;
      cs.pp.move(start, end); 
    }
    if (count == 150) {
      Matrix<2,1,double> start{15, 30}, end{-5, 20};
      log.warn() << "start trajectory from " << start << " to " << end;
      cs.pp.setStart(start);
      cs.pp.setJerk(2.0);
      cs.pp.setMaxVel(2.0);
      cs.pp.move(end);
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
