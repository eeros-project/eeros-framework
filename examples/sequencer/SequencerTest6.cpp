#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Periodic.hpp>
#include <eeros/task/Lambda.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

using namespace eeros;
using namespace eeros::task;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

// bool running = true;

class Robot {
public:
	Robot() : x(0), y(0) { }
	void move() {
		x += 0.005; if (x > posX) x = posX;
		y += 0.005; if (y > posY) y = posY;
	}
	double x, y;
	double posX, posY;
} r;

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties() : slSingle("single level") {
		addLevel(slSingle);
		slSingle.setLevelAction([&](SafetyContext* privateContext) {r.move();});
		setEntryLevel(slSingle);
	}
	virtual ~MySafetyProperties() { }
	SafetyLevel slSingle;
};

class StepX : public Step {
public:
	StepX(std::string name, Sequencer& sequencer, BaseSequence* caller) : Step(name, sequencer, caller) { }
	int action() {r.posX += 1;}
	bool checkExitCondition() {return r.x >= r.posX;}
};

class StepY : public Step {
public:
	StepY(std::string name, Sequencer& sequencer, BaseSequence* caller) : Step(name, sequencer, caller) { }
	int action() {r.posY += 1;}
	bool checkExitCondition() {return r.y >= r.posY;}
};

class SequenceA : public Sequence {
public:
	SequenceA(std::string name, Sequencer& sequencer, BaseSequence* caller) : Sequence(name, sequencer, caller), stepX("step X", seq, this) { }
	int action() {
		for (int i = 0; i < 5; i++) stepX();
// 		while (true) stepX();
	}
private:
	StepX stepX;
};

class SequenceB : public Sequence {
public:
	SequenceB(std::string name, Sequencer& sequencer, BaseSequence* caller) : Sequence(name, sequencer, caller), stepY("step Y", seq, this) { 
		setNonBlocking();
	}
	int action() {
		for (int i = 0; i < 10; i++) stepY();
	}
private:
	StepY stepY;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq), seqA("seq A", seq, this), seqB("seq B", seq, this) { 
		setNonBlocking();
	}
		
	int action() {
		seqA();
		seqB();
 		seqA();
		seqB.join();
	}
private:
	SequenceA seqA;
	SequenceB seqB;
};

void signalHandler(int signum) {
	Sequencer::instance().abort();;
	Executor::instance().stop();
}

int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	
	StreamLogWriter w(std::cout);
// 	w.show(LogLevel::TRACE);
	Logger::setDefaultWriter(&w);
	Logger log;
	
	log.info() << "Mock robot example started...";
	
	double period = 0.01;
	MySafetyProperties ssProperties;
	SafetySystem safetySys(ssProperties, period);

	auto& executor = Executor::instance();
	executor.setMainTask(safetySys);

	auto& sequencer = Sequencer::instance();
	MainSequence mainSeq("Main Sequence", sequencer);
	sequencer.addMainSequence(mainSeq);
	mainSeq.start();
	
	eeros::task::Lambda l1 ([&] () {log.warn() << "robot at " << r.x << "/" << r.y;});
	Periodic periodic("per1", 0.5, l1);
	executor.add(periodic);
	
	executor.run();
	
	// terminate sequencer 
// 	mainSeq.terminate();
	mainSeq.join();
	
	log.info() << "Mock robot example finished...";
}
