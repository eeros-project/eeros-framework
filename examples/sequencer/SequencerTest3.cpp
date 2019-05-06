#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequence* caller) : Sequence(name, caller, true), wait("wait", this) { }
	int action() {
		caller->resetTimeout(); 
		caller->setTimeoutTime(10);
	}
private:
	Wait wait;
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequence* caller) : Sequence(name, caller, true), stepB("step B", this) { }
	int action() {
		for (int i = 0; i < 5; i++) stepB(1);
// 		while (getRunningState() == SequenceState::running) stepB(1);
	}
private:
	Wait stepB;
};

class SequenceA : public Sequence {
public:
	SequenceA(std::string name, Sequence* caller) : Sequence(name, caller, true), stepA("step A", this), seqB("sequence B", this), eSeq("exception sequence", this) { 
		setTimeoutTime(3.5);
// 		setTimeoutExceptionSequence(eSeq);
		setTimeoutBehavior(SequenceProp::abort);
// 		setTimeoutBehavior(SequenceProp::restart);
// 		setTimeoutBehavior(SequenceProp::resume);
	}
	int action() {
		stepA(1);
		stepA(1);
		seqB();
		stepA(1);
		stepA(1);
	}
private:
	Wait stepA;
	SequenceB seqB;
	ExceptionSeq eSeq;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqA("sequence A", this) { }
		
	int action() {
		seqA();
	}
private:
	SequenceA seqA;
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
