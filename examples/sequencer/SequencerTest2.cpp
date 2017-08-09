#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& sequencer, BaseSequence* caller) : Step(name, sequencer, caller) { }
	int operator() () {return Step::start();}
	int action() { }
};

class StepB : public Step {
public:
	StepB(std::string name, Sequencer& sequencer, BaseSequence* caller) : Step(name, sequencer, caller) { }
	int operator() () {return Step::start();}
	int action() { }
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& sequencer, BaseSequence* caller) : Sequence(name, sequencer, caller), stepB("step B", seq, this) { 
		setNonBlocking();
	}
	int operator() () {return Sequence::start();}
	int action() {
		for (int i = 0; i < 5; i++) stepB();
	}
private:
	StepB stepB;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqB("seq B", seq, this), stepA("step A", seq, this) { 
		setNonBlocking();
	}
		
	int action() {
		for (int i = 0; i < 3; i++) stepA();
		seqB();
		for (int i = 0; i < 5; i++) stepA();
		seqB.join();
	}
private:
	StepA stepA;
	SequenceB seqB;
};

int main(int argc, char **argv) {
	StreamLogWriter w(std::cout);
// 	w.show(LogLevel::TRACE);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "Sequencer example started...";
	
	auto& sequencer = Sequencer::instance();
	MainSequence mainSeq("Main Sequence", sequencer);
	sequencer.addMainSequence(mainSeq);
	mainSeq.start();
	
	// Wait until sequencer terminates
	mainSeq.join();
	
	log.info() << "Simple Sequencer Example finished...";
}
