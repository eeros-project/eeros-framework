#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class MainSequence : public Sequence {
 public:
  MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), stepA("step A", this) { 
    setTimeoutTime(2.5);
    setTimeoutBehavior(SequenceProp::abort);
  }  
  int action() {
    for (int i = 0; i < 5; i++) stepA(1);
    return 0;
  }
  Wait stepA;
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
