#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

int count = 0;

class MyCondition : public Condition {
    bool validate() {return count > 2;}
};

class SequenceB : public Sequence {
public:
    SequenceB(std::string name, Sequence* caller, Monitor& m) : Sequence(name, caller, false), stepB("step B", this), m(m) { 
        addMonitor(&m);
    }
    int action() {
        for (int i = 0; i < 5; i++) stepB(1);
        return 0;
    }
private:
    Wait stepB;
    Monitor& m;
};

class MainSequence : public Sequence {
public:
    MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), m("myMonitor", this, cond, SequenceProp::abort), seqB("sequence B", this, m), stepA("step A", this) { 
        addMonitor(&m);
    }
        
    int action() {
        seqB();
        for (int i = 0; i < 5; i++) {
            stepA(1);
            count++;
        }
        seqB.waitAndTerminate();
        return 0;
    }
private:
    Monitor m;
    SequenceB seqB;
    MyCondition cond;
    Wait stepA;
};

void signalHandler(int signum) {
    Sequencer::instance().abort();
}

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    StreamLogWriter w(std::cout);
//  w.show(LogLevel::TRACE);
    Logger::setDefaultWriter(&w);
    Logger log;
    
    log.info() << "Sequencer example started...";
    
    auto& sequencer = Sequencer::instance();
    MainSequence mainSeq("Main Sequence", sequencer);
    sequencer.addSequence(mainSeq);
    mainSeq.start();
    
    sequencer.wait();
    log.info() << "Simple Sequencer Example finished...";
}
