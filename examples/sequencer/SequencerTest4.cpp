#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

int count = 0;

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), wait("wait E", seq, this) { }
	int action() {count = 0; wait(0.5);}
private:
	Wait wait;
};

class MyCondition : public Condition {
	bool validate() {return count > 2;}
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), stepA("step A", seq, this), eSeq("exception sequence", seq, this), m("myMonitor", this, cond, SequenceProp::resume, &eSeq) { 
		setTimeoutTime(7.0);
		setTimeoutBehavior(SequenceProp::abort);
		addMonitor(&m);
	}
		
	int action() {
		for (int i = 0; i < 5; i++) {
			stepA(2);
			count++;
		}
	}
private:
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
