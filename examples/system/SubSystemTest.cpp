#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/InputSub.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::math;

double period = 0.01;

class Subsystem: public Block {
 public: 
  Subsystem() : gain(3.0) {
    gain.getIn().connect(in);
    sum.getIn(0).connect(in);
    sum.getIn(1).connect(gain.getOut());
  }

  virtual void run()  {
    gain.run();
    sum.run();
  }
 
  virtual Input<Vector2>& getIn() {return in;}
  virtual Output<Vector2>& getOut() {return sum.getOut();}
 
private: 
  InputSub<Vector2> in;
  Gain<Vector2> gain;
  Sum<2, Vector2> sum;
};

class ControlSystem {
 public:
  ControlSystem() : c({1.0, 1.5}), g(2.0), td("td1", period, true) {
    sub.getIn().connect(c.getOut());
    g.getIn().connect(sub.getOut());
    td.addBlock(c);
    td.addBlock(sub);
    td.addBlock(g);
    Executor::instance().add(td);
  }

  Constant<Vector2> c;
  Subsystem sub;
  Gain<Vector2> g;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties() : slStart("start") {   
    addLevel(slStart);
    setEntryLevel(slStart); 
  }

  SafetyLevel slStart;
};

int main() {
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  
  log.info() << "Subsystem test started...";
  
  ControlSystem cs;
  TestSafetyProperties sp;
  SafetySystem ss(sp, period);
  
  Lambda l2 ([&] () { });
  Periodic periodic("p2", 0.5, l2);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
    log.info() << cs.g.getOut().getSignal();
  });
  
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();
  
  log.info() << "Test finished...";
}
