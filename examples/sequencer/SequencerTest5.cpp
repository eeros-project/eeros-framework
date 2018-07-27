#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <chrono>
#include <signal.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

int count = 0;

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now(); count++;}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 1.0;}
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

class MyCondition : public Condition {
	bool validate() {return count > 2;}
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& seq, BaseSequence* caller, Monitor& m) : Sequence(name, seq, caller, false), stepB("step B", seq, this), m(m) { 
		addMonitor(&m);
	}
	int action() {
		for (int i = 0; i < 5; i++) stepB();
	}
private:
	StepB stepB;
	Monitor& m;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), m("myMonitor", this, cond, SequenceProp::abort), seqB("sequence B", seq, this, m), stepA("step A", seq, this) { 
		addMonitor(&m);
	}
		
	int action() {
		seqB();
		for (int i = 0; i < 5; i++) stepA();
		seqB.waitAndTerminate();
	}
private:
	SequenceB seqB;
	StepA stepA;
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
	
	mainSeq.waitAndTerminate();
	log.info() << "Simple Sequencer Example finished...";
}
