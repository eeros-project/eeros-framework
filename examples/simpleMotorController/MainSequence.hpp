#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class Move : public Step {
public:
	Move(std::string name, Sequence* caller, MyControlSystem& cs) : Step(name, caller), cs(cs) { }
	int operator() (double pos) {this->pos = pos; return start();}
	int action() {
		cs.setpoint.setValue(pos);
	}
	double pos;
	MyControlSystem& cs;
};

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq, SafetySystem& safetySys, MySafetyProperties& safetyProp, MyControlSystem& cs, double angle) : 
					Sequence(name, seq), safetySys(safetySys), safetyProp(safetyProp), angle(angle), controlSys(cs), move("move", this, cs) {
		log.info() << "Sequence created: " << name;
	}
	int action() {
		while(safetySys.getCurrentLevel() < safetyProp.slMoving);
	
		angle = 0;
		while (Sequencer::running) {
			angle += 6.28 / 10;
			move(angle);
			sleep(1);
			log.info() << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
		}
	}
private:
	Move move;
	double angle;
	SafetySystem& safetySys;
	MyControlSystem& controlSys;
	MySafetyProperties& safetyProp;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_ 
