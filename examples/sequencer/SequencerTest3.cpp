#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Wait.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), stepB("step B", seq, this) { }
	int action() {
		for (int i = 0; i < 5; i++) stepB(1);
	}
private:
	Wait stepB;
};

class SequenceA : public Sequence {
public:
	SequenceA(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), stepA("step A", seq, this), seqB("sequence B", seq, this) { 
		setTimeoutTime(3.5);
		setTimeoutBehavior(SequenceProp::abort);
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
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqA("sequence A", seq, this) { }
		
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
