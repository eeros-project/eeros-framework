#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/task/Lambda.hpp>
#include "HalTest1.hpp"

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::task;
using namespace eeros::hal;
using namespace eeros::safety;
using namespace eeros::sequencer;

int main(int argc, char **argv){
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
    
  log.info() << "HAL simulator test started...";
  
  HAL& hal = HAL::instance();
  hal.readConfigFromFile(&argc, argv);
    
// 	hal.callOutputFeature("pwm1", "setPwmFrequency", 100.0);
    
  // Create safety and control system
  MyControlSystem cs(dt);
  MySafetyProperties safetyProperties;
  SafetySystem safetySystem(safetyProperties, dt);
    
  // Sequencer
  auto& sequencer = Sequencer::instance();
  MyMainSequence mainSequence(sequencer, cs);
  mainSequence();
    
  // Set executor and run
  auto &executor = Executor::instance();
  executor.setMainTask(safetySystem);
    
//   auto digOut = hal.getLogicOutput("dOut0", false);
  Lambda l1 ([&] () { });
  Periodic perLog("periodic log", 1, l1);
  perLog.monitors.push_back([&](PeriodicCounter &pc, Logger &log) {
//     log.warn() << cs.digOut0.getValue();
//     log.warn() << digOut->get();
    log.info() << cs.digIn0.getOut().getSignal();
    log.info() << cs.digIn1.getOut().getSignal();
    log.info() << cs.anIn0.getOut().getSignal();
    log.info() << cs.anIn2.getOut().getSignal();
  });
  executor.add(perLog);
  executor.run();
    
  sequencer.wait();
  log.info() << "end...";
  return 0;
}