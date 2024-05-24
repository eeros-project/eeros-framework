#include "CANexample2.hpp"
#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::sequencer;
 
void signalHandler(int signum) {
  SafetySystem::exitHandler();
}

int main(int argc, char **argv) {
  double dt = 0.1;
  signal(SIGINT, signalHandler);
  
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');

  log.info() << "CAN test start";

  CANopen co("can0");
  ControlSystem cs(co, dt);
  MySafetyProperties sp;
  SafetySystem ss(sp, dt);
  auto& sequencer = Sequencer::instance();
  MainSequence mainSeq("Main Sequence", sequencer, cs, ss, sp, co);
  mainSeq();
    
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  executor.run();

  sequencer.wait();
  log.info() << "CAN test end";	
  return 0;
}
