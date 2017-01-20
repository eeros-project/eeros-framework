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
	std::cout << "sequence run" << std::endl;
	
	while(safetySys.getCurrentLevel() < safetyProp.moving){
		if(isTerminating()) break;
		usleep(10000);
	}
	
	
	while(!isTerminating()){
		double a = 0;
		sleep(5);
		for(int i = 0; i < 10; i++) {
			log.info() << "[" << getName() << "] " << "setting angle to " << a;
			controlSys.setpoint.setValue(a);
// 			controlSys.dac.getIn().getSignal().setValue(2.5);
			a += angle;
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
		
		sleep(1);
		log.info() << "[" << getName() << "] " << "setting angle to " << -3.14;
		controlSys.setpoint.setValue(-3.14);
// 		controlSys.dac.getIn().getSignal().setValue(-2.5);
			
		sleep(1);
		log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
		sleep(1);
	}
}

bool SequenceA::isTerminating() {
	return sequencer->getState() == state::terminating;
}

void SequenceA::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	safetySys.triggerEvent(safetyProp.stopMoving);
	sleep(1);
	log.info() << "[" << getName() << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
// 	return safetySys.getCurrentLevel() >= safetyProp.moving;
	return true;
}

// bool SequenceA::checkPostCondition() {
// 	log.info() << "[" << getName() << "] " << "Checking postcondition...";
// 	return true;
// }
