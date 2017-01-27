#include "SequenceA.hpp"
#include "MyControlSystem.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>
#include <iostream>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, eeros::sequencer::Sequencer& sequencer, SafetySystem& safetySys, MySafetyProperties& properties, MyControlSystem& controlSys, double angle) : 
					Sequence<void>(name, &sequencer), safetySys(safetySys), safetyProp(properties), angle(angle), controlSys(controlSys) {
	log.info() << "Sequence created: " << name;
}

void SequenceA::run() {
	log.info() << "[" + getName() + " started]";
	
	while(safetySys.getCurrentLevel() < safetyProp.moving){
		if(isTerminating()) break;
		usleep(10000);
	}
	
	
	while(!isTerminating()){
		double a = 0;
		for(int i = 0; (i < 10) && (!isTerminating()) ; i++) {
			log.info() << "[" << getName() << "] " << "setting angle to " << a;
			controlSys.setpoint.setValue(a);
			a += angle;
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
		
		if(!isTerminating()){
			sleep(1);
			log.info() << "[" << getName() << "] " << "setting angle to " << -3.14;
			controlSys.setpoint.setValue(-3.14);
				
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
	}
}

inline bool SequenceA::isTerminating() {
	return sequencer->getState() == state::terminating;
}

void SequenceA::exit() {
	log.info() << "[" << getName() << "] " << "Exit done.";
}

bool SequenceA::checkPreCondition() {
	log.info() << "[" << getName() << "] " << "Checking precondition...";
	return true;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << getName() << "] " << "Checking postcondition...";
	return true;
}
