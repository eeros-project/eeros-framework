#include <eeros/safety/SafetySystem.hpp>
#include "ParserTestMainSequence.hpp"
#include "../control/ParserTestControlSystem.hpp"
#include <unistd.h>
#include <iostream>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::hal;


ParserTestMainSequence::ParserTestMainSequence(Sequencer* sequencer, ParserTestControlSystem* controlSys, SafetySystem* safetySys) :
							Sequence<void>("main", sequencer), controlSys(controlSys), safetySys(safetySys) {
	
}

bool ParserTestMainSequence::checkPreCondition() {
	return true;
}

void ParserTestMainSequence::run() {
	log.trace() << "[ Main Sequence Started ]";
	
	log.info() << "Starting...";
	for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
	  
		if(i%4 == 0){
// 			controlSys->dac1.getIn().getSignal().setValue(2.5);
// 			controlSys->setPos.getOut().getSignal().setValue(true);
			controlSys->io1.getIn().getSignal().setValue(true);
		}
		else{
// 			controlSys->dac1.getIn().getSignal().setValue(-2.5);
// 			controlSys->setPos.getOut().getSignal().setValue(false);
			controlSys->io1.getIn().getSignal().setValue(false);
		}
		usleep(100000);
	}
}

void ParserTestMainSequence::exit() {
	log.info() << "[ Exit Main Sequence ]";
}

inline bool ParserTestMainSequence::isTerminating() {
	return sequencer->getState() == state::terminating;
}