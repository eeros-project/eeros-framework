#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class ExceptionSeq : public Sequence {
public:
  ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true), wait("wait", this) { }
  int action() {wait(3); return 0;}
  Wait wait;
};

class SequenceS : public Sequence {
public:
  SequenceS(std::string name, Sequencer& seq, Sequence* caller) : Sequence(name, caller, false), stepB("step B", this) { }
  int action() {
    for (int i = 0; i < 5; i++) stepB(1);
    return 0;
  }
  Wait stepB;
};

class MainSequence : public Sequence {
public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqS("seq S", seq, this), stepA("step A", this), eSeq("exception sequence", this) {
    setTimeoutTime(4.5);
    setTimeoutExceptionSequence(eSeq);
    setTimeoutBehavior(SequenceProp::abort);
  }
    
  int action() {
    for (int i = 0; i < 3; i++) {
      stepA(1);
    }
    seqS();
    for (int i = 0; i < 3; i++) {
      stepA(1);
    }
    seqS.wait();
    return 0;
  }
  SequenceS seqS;
  Wait stepA;
  ExceptionSeq eSeq;
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
  log.info() << "Simple Sequencer Example finished...";
}
