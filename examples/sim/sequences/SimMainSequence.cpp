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


SimMainSequence::SimMainSequence(Sequencer* sequencer, SimControlSystem* controlSys, SafetySystem* safetySys) :
							Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	
}

bool SimMainSequence::checkPreCondition() {
	return true;
}

void SimMainSequence::run() {
	bool set = false;
	log.trace() << "[ Main Sequence Started ]";
	
	log.info() << "Starting...";
	for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
		
		if(i%5 == 0){
			log.info() << "simOut_in0: " << controlSys->simOut_in0.getOut().getSignal().getValue() << "\tsimIn_in1: " << simIn_in1.get();
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
		
		usleep(100000);
	}
}

void SimMainSequence::exit() {
	log.info() << "[ Exit Main Sequence ]";
}

inline bool SimMainSequence::isTerminating() {
	return sequencer->getState() == state::terminating;
}