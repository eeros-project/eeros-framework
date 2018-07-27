#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

class StepB : public Step {
public:
	StepB(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now();}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 1.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), stepB("step B", seq, this) { }
	int action() {
		for (int i = 0; i < 5; i++) stepB();
	}
private:
	StepB stepB;
};

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now();}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 1.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class SequenceA : public Sequence {
public:
	SequenceA(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller, true), stepA("step A", seq, this), seqB("sequence B", seq, this) { 
		setTimeoutTime(3.5);
		setTimeoutBehavior(SequenceProp::abort);
	}
	int action() {
		stepA();
		stepA();
		seqB();
		stepA();
		stepA();
	}
private:
	StepA stepA;
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
	
	mainSeq.waitAndTerminate();
	
	log.info() << "Simple Sequencer Example finished...";
}
