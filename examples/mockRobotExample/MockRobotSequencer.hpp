#ifndef MOCK_ROBOT_SEQUENCER_HPP_
#define MOCK_ROBOT_SEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include "MockRobotSafetyProperties.hpp"
#include "MockRobotControlSystem.hpp"

using namespace eeros;
using namespace eeros::task;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class MoveUp : public Step {
public:
	MoveUp(std::string name, Sequencer& sequencer, BaseSequence* caller, MockRobotControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int action() {
		cs.setpointX.setValue(0.7);
		cs.setpointY.setValue(0.3);
	}
	bool checkExitCondition() {return cs.iX.getOut().getSignal().getValue() >= 5.0;}
private:
	MockRobotControlSystem& cs;
};

class MoveDown : public Step {
public:
	MoveDown(std::string name, Sequencer& sequencer, BaseSequence* caller, MockRobotControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int action() {
		cs.setpointX.setValue(-0.6);
		cs.setpointY.setValue(-0.3);
	}
	bool checkExitCondition() {return cs.iX.getOut().getSignal().getValue() <= -5.0;}
private:
	MockRobotControlSystem& cs;
};

class UpAndDownSequence : public Sequence {
public:
	UpAndDownSequence(std::string name, Sequencer& seq, MockRobotControlSystem& cs) : Sequence(name, seq), cs(cs), moveUp("move up", seq, this, cs), moveDown("move down", seq, this, cs) { 
		setNonBlocking();
	}
		
	int action() {
		while (Sequencer::running) {
			moveUp();
			moveDown();
		}
	}
private:
	MockRobotControlSystem& cs;
	MoveUp moveUp;
	MoveDown moveDown;
};

class HomingSequence : public Sequence {
public:
	HomingSequence(std::string name, Sequencer& seq, MockRobotControlSystem& cs) : Sequence(name, seq), cs(cs) { 
		setNonBlocking();
	}
		
	int action() {
		cs.setpointX.setValue(0.1);
		cs.setpointY.setValue(0.1);
	}
	bool checkExitCondition() {return cs.iX.getOut().getSignal().getValue() >= 1.0 && cs.iY.getOut().getSignal().getValue() >= 1.0;}
private:
	MockRobotControlSystem& cs;
};

#endif // MOCK_ROBOT_SEQUENCER_HPP_