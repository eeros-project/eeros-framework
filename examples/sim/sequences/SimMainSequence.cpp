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
			log.info() << "in0: " << controlSys->in0.getOut().getSignal().getValue();
		}
		
		if(i%50 == 0){
			if(set){
				log.info() << "set false";
				controlSys->out0.getIn().getSignal().setValue(false);
				set = false;
			}
			else{
				log.info() << "set true";
				controlSys->out0.getIn().getSignal().setValue(true);
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