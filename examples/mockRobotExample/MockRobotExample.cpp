#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include "MockRobotSafetyProperties.hpp"
#include "MockRobotControlSystem.hpp"
#include "MockRobotSequencer.hpp"

using namespace eeros;
using namespace eeros::task;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;


void signalHandler(int signum) {
  SafetySystem::exitHandler();
  Sequencer::instance().abort();;
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);
  
  StreamLogWriter w(std::cout);
//   w.show(LogLevel::TRACE);
  Logger::setDefaultWriter(&w);
  Logger log;
  
  log.info() << "Mock robot example started...";
  
  double period = 0.01;
  MockRobotControlSystem cs(period);
  MockRobotSafetyProperties sp(cs, period);
  SafetySystem ss(sp, period);

  auto& executor = Executor::instance();
  executor.setMainTask(ss);

  auto& sequencer = Sequencer::instance();
  MainSequence mainSeq("Main Sequence", sequencer, cs, ss, sp);
  mainSeq();
  
  eeros::task::Lambda l1 ([&] () {log.warn() << "robot at " << cs.iX.getOut().getSignal().getValue() << "/" << cs.iY.getOut().getSignal().getValue();});
  Periodic periodic("per1", 0.5, l1);
  executor.add(periodic);
  executor.run();
  
  sequencer.wait();
  log.info() << "Mock robot example finished...";
}
