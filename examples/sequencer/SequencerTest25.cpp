#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class ExceptionSeq : public Sequence {
public:
  ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true), wait("wait", this), caller(caller) { }
  int action() {
    wait(3); 
    caller->resetTimeout();
    return 0;
  }
  Wait wait;
  Sequence* caller;
};

class SequenceS : public Sequence {
public:
  SequenceS(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, true), stepB("step B", this), eSeq("exception sequence", this) {
    setTimeoutTime(2.5);
    setTimeoutExceptionSequence(eSeq);
    setTimeoutBehavior(SequenceProp::resume);
  }
  int action() {
    for (int i = 0; i < 5; i++) stepB(1);
    return 0;
  }
  Wait stepB;
  ExceptionSeq eSeq;
};

class MainSequence : public Sequence {
public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqS("seq S", seq, this), stepA("step A", this) { }
    
  int action() {
    for (int i = 0; i < 3; i++) {
      stepA(1);
    }
    seqS();
    for (int i = 0; i < 3; i++) {
      stepA(1);
    }
    return 0;
  }
  SequenceS seqS;
  Wait stepA;
};

void signalHandler(int signum) {
  Sequencer::instance().abort();
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger('M');
  log.info() << "Sequencer example started...";
  
  auto& sequencer = Sequencer::instance();
  MainSequence mainSeq("Main Sequence", sequencer);
  mainSeq();
  sequencer.wait();
  log.info() << "Simple Sequencer Example finished...";
}
