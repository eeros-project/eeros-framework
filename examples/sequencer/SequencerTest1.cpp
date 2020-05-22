#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>
#include <signal.h>
#include <chrono>

using namespace eeros::sequencer;
using namespace eeros::logger;

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true), wait("wait", this) { }
	int action() {
		wait(3);
// 		caller->resetTimeout(); 
// 		caller->setTimeoutTime(4.5);
// 		caller->setTimeoutBehavior(SequenceProp::abort);
        return 0;
	}
private:
	Wait wait;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), stepA("step A", this), eSeq("exception sequence", this) { 
		setTimeoutTime(2.5);
		setTimeoutExceptionSequence(eSeq);
// 		setTimeoutBehavior(SequenceProp::resume);
// 		setTimeoutBehavior(SequenceProp::abort);
		setTimeoutBehavior(SequenceProp::restart);
	}
		
	int action() {
		for (int i = 0; i < 5; i++) stepA(1);
// 		while (getRunningState() == SequenceState::running) stepA(1);
        return 0;
	}
private:
	Wait stepA;
	ExceptionSeq eSeq;
};

void signalHandler(int signum) {
	Sequencer::instance().abort();
}

int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	StreamLogWriter w(std::cout);
// 	w.show(LogLevel::TRACE);
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
