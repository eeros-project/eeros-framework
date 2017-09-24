#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <signal.h>
#include <chrono>

using namespace eeros::sequencer;
using namespace eeros::logger;

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now();}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 2.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class StepB : public Step {
public:
	StepB(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now();}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 1.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now();}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 3.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller), stepB("step B", seq, this), eSeq("exception sequence", seq, this) { 
		setNonBlocking();
		setTimeoutTime(2.5);
		setTimeoutExceptionSequence(*(seq.getSequenceByName("exception sequence")));
		setTimeoutBehavior(SequenceProp::abort);
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
	
	sequencer.join();	// wait until sequencer terminates
	
	log.info() << "Simple Sequencer Example finished...";
}
