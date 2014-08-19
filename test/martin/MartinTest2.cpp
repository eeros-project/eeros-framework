#include <iostream>
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/ui/CursesUI.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;
using namespace eeros::ui;

class Robot {
public:
	Robot() : x(0), y(0), z(0) { }
	
	void moveXY(double x, double y) {
		this->x = x; this->y = y;
		sleep(1);
	}
	
	void moveZ(double z) {
		this->z = z;
		sleep(1);
	}
	
	double getPosX() {
		return x;
	}
	
	double getPosY() {
		return y;
	}
	
	double getPosZ() {
		return z;
	}
	
	bool tcpSensor() {
		return static_cast<int>(x + y) % 2;
	}
	
	double x, y, z;
};

class MoveSequence : public Sequence<void, double, double> {
public:
	MoveSequence(std::string name, Sequencer* sequencer, Robot* robot) : Sequence<void, double, double>(name, sequencer), robot(robot) { }
	
	void run(double x, double y) {
		log.trace() << "Sequence '" << getName() << "': Moving robot to position x = " << x << " / y = " << y;
		robot->moveZ(5);
		robot->moveXY(x, y);
		robot->moveZ(0);
	}
	
private:
	Robot* robot;
};

class DetectSequence : public Sequence<bool, double> {
public:
	DetectSequence(std::string name, Sequencer* sequencer, Robot* robot) : Sequence<bool, double>(name, sequencer), robot(robot) { }
		
	virtual bool checkPreCondition() {
		if(robot->getPosZ() > -0.1) return true;
		return false;
	}
		
	bool run(double z) {
		log.trace() << "Sequence '" << getName() << "': Detecting object";
		robot->moveZ(z);
		bool res = robot->tcpSensor();
		robot->moveZ(0);
		return res;
	}
	
private:
	Robot* robot;
};

class DummySequence : public Sequence<int> {
public:
	DummySequence(std::string name, Sequencer* sequencer) : Sequence<int>(name, sequencer), c(0) { }
	
	int run() {
		log.trace() << "Sequence '" << getName() << "'";
		return c++;
	}
	
private:
	int c;
};

class MainSequence : public Sequence<> {
public:
	MainSequence(std::string name, Sequencer* sequencer, Robot* robot) : Sequence<>(name, sequencer), robot(robot), move("move", sequencer, robot), detect("detect", sequencer, robot), dummy("dummy", sequencer) {
		log.trace() << "Creating new sequence '" << name << "'...";
	}
		
	void run() {
		log.trace() << "Sequence '" << getName() << "': " << "Move to position A";
		move(2, 3);
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Detecting object at position A";
		auto resA = detect(-2);
		if(resA.result) {
			if(resA.value) log.info() << "Sequence '" << getName() << "': Object detected!";
			else log.info() << "Sequence '" << getName() << "': No object detected!";
		}
		else {
			log.warn() << "Sequence '" << getName() << "': detection failed!";
		}
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Move to position B";
		move(8, 8);
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Detecting object at position B";
		auto resB = detect(-5);
		if(resB.result) {
			if(resB.value) log.info() << "Sequence '" << getName() << "': Object detected!";
			else log.info() << "Sequence '" << getName() << "': No object detected!";
		}
		else {
			log.warn() << "Sequence '" << getName() << "': detection failed!";
		}
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Calling dummy sequence";
		dummy();
	}
	
	void init() {
		log.trace() << "Sequence '" << getName() << "': " << "Init started";
		sleep(1);
		log.trace() << "Sequence '" << getName() << "': " << "Init done";
	}
	
	void exit() {
		log.trace() << "Sequence '" << getName() << "': " << "Exit started";
		sleep(1);
		log.trace() << "Sequence '" << getName() << "': " << "Exit done";
	}

private:
	Robot* robot;
	MoveSequence move;
	DetectSequence detect;
	DummySequence dummy;
};


int main() {
	// Create and initialize logger
	SysLogWriter w("martinTest2");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	// Create dummy robot
	Robot robot;
	
	// Create sequencer and UI
	Sequencer sequencer;
	CursesUI ui(sequencer);
	MainSequence mainSeq("Main Sequence", &sequencer, &robot);
	
	ui.dispay();
	sequencer.stepMode();
	sequencer.start(&mainSeq);
	
	// Wait until sequencer terminates
	sequencer.join();

	log.info() << "Martin Test 2 finished...";
}
