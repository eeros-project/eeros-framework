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
	ExceptionSeq(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), wait("wait", seq, this) { }
	int action() {wait(3);}
private:
	Wait wait;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), stepA("step A", seq, this), eSeq("exception sequence", seq, this) { 
		setTimeoutTime(2.5);
		setTimeoutExceptionSequence(eSeq);
// 		setTimeoutBehavior(SequenceProp::resume);
// 		setTimeoutBehavior(SequenceProp::abort);
		setTimeoutBehavior(SequenceProp::restart);
	}
		
	int action() {
		auto& sequencer = Sequencer::instance();
		if (Sequencer::running)
			for (int i = 0; i < 5; i++) stepA(1);
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
