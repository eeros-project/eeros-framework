#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/Version.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/core/System.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>

/*
* example with no main task and one harmonic task
*/ 

using namespace eeros::logger;
using namespace eeros;

int main() {
  const double dt = 0.01;
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');

  log.trace() << "harmonic tasks example 2";

  task::Lambda l1 ([&] () { });
  task::Periodic harmonicTask("harmonic task", 2, l1);
  
  harmonicTask.addDefaultMonitor();
  harmonicTask.monitors.push_back([](eeros::PeriodicCounter &c, Logger &log){
    log.info() << "harmonic task: period max: " << c.period.max << "   period min: " << c.period.min << "   period mean: " << c.period.mean;
  });
  
  Executor& executor = Executor::instance();
  executor.setExecutorPeriod(dt);
  executor.add(harmonicTask);

  executor.run();

  return 0;
}
