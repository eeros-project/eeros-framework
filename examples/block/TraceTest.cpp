#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Trace.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>
#include <signal.h>
#include <iostream>
#include <fstream>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::task;
using namespace eeros::math;

double period = 0.1;
double traceLen = 64;

class ControlSystem {
 public:
  ControlSystem() : c(0.1), trace1(traceLen), trace2(traceLen), td("td1", period, true) {
    i.getIn().connect(c.getOut());
    i.setInitCondition(Vector3{0, 1.0, 2.0});
    i.enable();
    c.setName("const block");
    i.setName("integrator");
    trace1.setName("trace constant output");
    trace2.setName("trace integrator output");
    trace1.getIn().connect(c.getOut());
    trace2.getIn().connect(i.getOut());
    td.addBlock(c);
    td.addBlock(i);
    td.addBlock(trace1);
    td.addBlock(trace2);
    Executor::instance().add(td);
  }
  Constant<Vector3> c;
  I<Vector3> i;
  Trace<Vector3> trace1, trace2;
  TimeDomain td;
};

class TestSafetyProperties : public SafetyProperties {
 public:
  TestSafetyProperties(ControlSystem& cs) 
      : slOff("off"), slRunning("running"), seShutDown("switching off"), 
        tw(cs.trace2, "/mnt/ramdisk/ctrlData1.txt"), log(logger::Logger::getLogger()) {
    addLevel(slOff);
    addLevel(slRunning);
    slRunning.addEvent(seShutDown, slOff, kPrivateEvent);
    slOff.setLevelAction([&](SafetyContext* privateContext) {Executor::stop();});
    slRunning.setLevelAction([&](SafetyContext* privateContext) {
      if (slRunning.getNofActivations() == (uint32_t)(3 / period)) {	// start after 3s
        log.info() << "start tracing";
        cs.trace1.enable();
        cs.trace2.enable();
      }
      if (slRunning.getNofActivations() % (int)(30 / period) == 0) {	// write to log file every 30s
        tw.write();
      }
    });
    exitFunction = [&](SafetyContext* privateContext) {privateContext->triggerEvent(seShutDown);};
    setEntryLevel(slRunning);	
  };
  SafetyLevel slOff, slRunning;
  SafetyEvent seShutDown;
  TraceWriter<Vector3> tw;
  Logger log;
};

void signalHandler(int signum) {
  SafetySystem::exitHandler();
}

int main() {
  signal(SIGINT, signalHandler);

  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');
  log.show();
  
  log.info() << "Trace test started...";
  
  ControlSystem cs;
  TestSafetyProperties sp(cs);
  SafetySystem ss(sp, period);
  
  // create periodic function for logging
  Lambda l1 ([&] () { });
  Periodic periodic("per1", 0.5, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.i.getOut().getSignal();
  });
  
  // Create and run executor
  auto& executor = eeros::Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();
  
  std::string fileName = "/mnt/ramdisk/ctrlData2.txt";
  log.info() << "start writing file " << fileName;
  uint64_t start = eeros::System::getTimeNs();
  std::ofstream file;
  file.open(fileName, std::ios::trunc);
  timestamp_t* timeStampBuf = cs.trace1.getTimestampTrace();
  Vector3* buf1 = cs.trace1.getTrace();
  Vector3* buf2 = cs.trace2.getTrace();
  for (uint32_t i = 0; i < cs.trace1.getSize(); i++) file << timeStampBuf[i] << " " << buf1[i] << " " << buf2[i] << std::endl;
  file.close();
  uint64_t stop = eeros::System::getTimeNs();
  log.info() << "file written in " << (stop - start) << "ns";

  log.info() << "Test finished...";
}
