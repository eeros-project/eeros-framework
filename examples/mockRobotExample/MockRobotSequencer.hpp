#ifndef MOCK_ROBOT_SEQUENCER_HPP_
#define MOCK_ROBOT_SEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Wait.hpp>
#include "MockRobotControlSystem.hpp"
#include "MockRobotSafetyProperties.hpp"

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class MoveUp : public Step {
public:
	MoveUp(std::string name, Sequence* caller, MockRobotControlSystem& cs) : Step(name, caller), cs(cs) { }
	int action() {
		cs.setpointX.setValue(0.7);
		cs.setpointY.setValue(0.3);
        return 0;
	}
	bool checkExitCondition() {return cs.iX.getOut().getSignal().getValue() >= 5.0;}
private:
	MockRobotControlSystem& cs;
};

class MoveDown : public Step {
public:
	MoveDown(std::string name, Sequence* caller, MockRobotControlSystem& cs) : Step(name, caller), cs(cs) { }
	int action() {
		cs.setpointX.setValue(-0.6);
		cs.setpointY.setValue(-0.3);
        return 0;
	}
	bool checkExitCondition() {return cs.iX.getOut().getSignal().getValue() <= -5.0;}
private:
	MockRobotControlSystem& cs;
};

class UpAndDownSequence : public Sequence {
public:
	UpAndDownSequence(std::string name, Sequence* caller, MockRobotControlSystem& cs) : 
		Sequence(name, caller, true), 
		moveUp("move up", this, cs), 
		moveDown("move down", this, cs) { }
		
	int action() {
		while (Sequencer::running) {
			moveUp();
			moveDown();
		}
		return 0;
	}
private:
	MoveUp moveUp;
	MoveDown moveDown;
};

class HomingSequence : public Sequence {
public:
	HomingSequence(std::string name, Sequence* caller, MockRobotControlSystem& cs, SafetySystem& ss, MockRobotSafetyProperties& sp) : 
		Sequence(name, caller, true), 
		cs(cs), ss(ss), sp(sp) { }
		
	int action() {
		cs.setpointX.setValue(0.1);
		cs.setpointY.setValue(0.1);
        return 0;
	}
	bool checkExitCondition() {
		bool done = cs.iX.getOut().getSignal().getValue() >= 1.0 && cs.iY.getOut().getSignal().getValue() >= 1.0;
		if (cs.iX.getOut().getSignal().getValue() >= 1.0) cs.setpointX.setValue(0);
		if (cs.iY.getOut().getSignal().getValue() >= 1.0) cs.setpointY.setValue(0);
		if (done) ss.triggerEvent(sp.homingDone);
		return done;
	}
private:
	MockRobotControlSystem& cs;
	SafetySystem& ss;
	MockRobotSafetyProperties& sp;
};


class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq, MockRobotControlSystem& cs, SafetySystem& ss, MockRobotSafetyProperties& sp) : 
		Sequence(name, seq), ss(ss), sp(sp),
		homing("Homing Sequence", this, cs, ss, sp),  
		upDown("UpDown Sequence", this, cs),
		wait("wait", this) { }
		
	int action() {
		while(Sequencer::running) {
			if (ss.getCurrentLevel() == sp.slHoming) {
				homing();
			} else if(ss.getCurrentLevel() == sp.slReady) {
				wait(2);
				ss.triggerEvent(sp.startMoving);
			} else if(ss.getCurrentLevel() == sp.slMoving) {
				upDown();
			}
			wait(0.1);
		}
		return 0;
	}
private:
	SafetySystem& ss;
	MockRobotSafetyProperties& sp;
	HomingSequence homing;
	UpAndDownSequence upDown;
	Wait wait;
};


#endif // MOCK_ROBOT_SEQUENCER_HPP_