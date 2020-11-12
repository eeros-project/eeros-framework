#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

int count;

class ExceptionSeq : public Sequence {
 public:
  ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true), wait("wait", this) { }
  int action() {
    count = 0; 
    wait(0.5); 
    return 0;
  }
  Wait wait;
};

class MyCondition : public Condition {
  bool validate() {return count > 2;}
};

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq) 
      : Sequence(name, seq), stepA("step A", this), eSeq("exception sequence", this), 
        m("myMonitor", this, cond, SequenceProp::resume, &eSeq) { 
    setTimeoutTime(10.0);
    setTimeoutExceptionSequence(eSeq);
    setTimeoutBehavior(SequenceProp::abort);
    addMonitor(&m);
  }
    
  int action() {
    count = 0;
    for (int i = 0; i < 10; i++) {
      stepA(2);
      count++;
    }
    return 0;
  }

  Wait stepA;
  ExceptionSeq eSeq;
  MyCondition cond;
  Monitor m;
};

void signalHandler(int signum) {
  Sequencer::instance().abort();
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "Sequencer example started...";
  
  auto& sequencer = Sequencer::instance();
  MainSequence mainSeq("Main Sequence", sequencer);
  mainSeq();
  sequencer.wait();
  log.info() << "Simple sequencer example finished...";
}
