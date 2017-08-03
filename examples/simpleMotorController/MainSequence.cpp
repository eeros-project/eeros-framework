#include "MainSequence.hpp"
#include "MyControlSystem.hpp"
#include <unistd.h>
#include <iostream>


MainSequence::MainSequence(std::string name, Sequencer& sequencer, SafetySystem& safetySys, MySafetyProperties& safetyProp, MyControlSystem& controlSys, double angle) : 
					Sequence(name, sequencer), safetySys(safetySys), safetyProp(safetyProp), angle(angle), controlSys(controlSys) {
	log.info() << "Sequence created: " << name;
}

int MainSequence::action() {
	while(safetySys.getCurrentLevel() < safetyProp.slMoving);
	
	while(true) {
		double a = 0;
		for(int i = 0; (i < 10); i++) {
			log.info() << "[" << getName() << "] " << "setting angle to " << a;
			controlSys.setpoint.setValue(a);
			a += angle;
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
		
// 		if(!isTerminating()){
		if(true){
			sleep(1);
			log.info() << "[" << getName() << "] " << "setting angle to " << -3.14;
			controlSys.setpoint.setValue(-3.14);
				
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
	}
}

