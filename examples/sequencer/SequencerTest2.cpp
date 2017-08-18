#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {log.info() << "do step A";}
};

class StepB : public Step {
public:
	StepB(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {log.info() << "do step B"; sleep(1);}
};

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller) { }
	int action() {
		log.info() << "do exception sequence";
	}
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller), stepB("step B", seq, this), eSeq("exception sequence", seq, this) { 
		setNonBlocking();
		setTimeoutTime(2.5);
		setTimeoutExceptionSequence(*(seq.getSequenceByName("exception sequence")));
// 		setTimeoutBehavior(SequenceProp::nothing);
		setTimeoutBehavior(SequenceProp::abortOwner);
// 		setTimeoutBehavior(SequenceProp::restartCallerOfOwner);
	}
	int action() {
		for (int i = 0; i < 5; i++) stepB();
	}
private:
	StepB stepB;
	ExceptionSeq eSeq;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqB("seq B", seq, this), stepA("step A", seq, this) { 
		setNonBlocking();
// 		setTimeoutTime(3.0);
// 		setTimeoutBehavior(SequenceProp::abortOwner);
// 		setTimeoutExceptionSequence(&stepA);
// 		seqB.setTimeoutTime(3.0);
// 		seqB.setTimeoutBehavior(SequenceProp::abortOwner);
// 		seqB.setTimeoutExceptionSequence(&stepA);
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
