#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <chrono>

using namespace eeros::sequencer;
using namespace eeros::logger;

int count = 0;

class StepA : public Step {
public:
	StepA(std::string name, Sequencer& seq, BaseSequence* caller) : Step(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now(); count++;}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 2.0;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class ExceptionSeq : public Sequence {
public:
	ExceptionSeq(std::string name, Sequencer& seq, BaseSequence* caller) : Sequence(name, seq, caller) { }
	int action() {time = std::chrono::steady_clock::now(); count = 0;}
	bool checkExitCondition() {return ((std::chrono::duration<double>)(std::chrono::steady_clock::now() - time)).count() > 0.5;}
private:
	std::chrono::time_point<std::chrono::steady_clock> time;
};

class MyCondition : public Condition {
	bool validate() {return count > 2;}
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), stepA("step A", seq, this), eSeq("exception sequence", seq, this), m("myMonitor", this, cond, SequenceProp::resume, &eSeq) { 
		setNonBlocking();
		setTimeoutTime(7.0);
		setTimeoutBehavior(SequenceProp::abort);
		addMonitor(&m);
	}
		
	int action() {
		for (int i = 0; i < 5; i++) stepA();
	}
private:
	StepA stepA;
	ExceptionSeq eSeq;
	MyCondition cond;
	Monitor m;
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
