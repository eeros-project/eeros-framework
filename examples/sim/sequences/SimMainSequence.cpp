#include <eeros/safety/SafetySystem.hpp>
#include "SimMainSequence.hpp"
#include "../control/SimControlSystem.hpp"
#include <unistd.h>
#include <iostream>

#include <eeros/hal/Output.hpp>
#include <dlfcn.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;


SimMainSequence::SimMainSequence(Sequencer* sequencer, SimControlSystem* controlSys, SafetySystem* safetySys, SimSafetyProperties* properties ) :
							Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys), safetyProp(properties) {
	
}

bool SimMainSequence::checkPreCondition() {
	return true;
}

void SimMainSequence::run() {
	while(safetySys->getCurrentLevel() < safetyProp->slRunning){
		if(isTerminating()) break;
		usleep(10000);
	}
	
	bool set = false;
	bool toggleAnalog = false;
	log.trace() << "[ Main Sequence Started ]";
	
	log.info() << "Starting...";
	for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
		
		if(i%5 == 0){
			log.info() << "simOut_in0:\t" << controlSys->simOut_in0.getOut().getSignal().getValue() << "\tsimIn_in1:\t" << simIn_in1.get();
			log.info() << "aInTest0:\t" << controlSys->aInTest0.getOut().getSignal().getValue() << 
			"\tsimIn_aIn2:\t" << controlSys->aIn2.getOut().getSignal().getValue();
		}
		
		if(i%50 == 0){
			if(set){
				log.info() << "set false";
				controlSys->simOut_out0.getIn().getSignal().setValue(false);
				simIn_out1.set(false);
				set = false;
			}
			else{
				log.info() << "set true";
				controlSys->simOut_out0.getIn().getSignal().setValue(true);
				simIn_out1.set(true);
				set = true;
			}
				
		}
		if(i%100 == 0){
			if(toggleAnalog){
				log.info() << "set aOut to 6, test aOut2 to -3.2";
// 				controlSys->aOut0.getIn().getSignal().setValue(6.0);
				aOut0.set(6.0);
				controlSys->aOutTest2.getIn().getSignal().setValue(-3.2);
// 				aOutTest2.set(-3.0);
				toggleAnalog = false;
			}
			else{
				log.info() << "set aOut to -2, test aOut2 to 7.6";
// 				controlSys->aOut0.getIn().getSignal().setValue(-2.0);
				aOut0.set(-2.0);
				controlSys->aOutTest2.getIn().getSignal().setValue(7.6);
// 				aOutTest2.set(7.5);
				toggleAnalog = true;
			}
		}
		
		usleep(100000);
	}
}

void SimMainSequence::exit() {
	log.info() << "[ Exit Main Sequence ]";
}

inline bool SimMainSequence::isTerminating() {
	return sequencer->getState() == state::terminating;
}