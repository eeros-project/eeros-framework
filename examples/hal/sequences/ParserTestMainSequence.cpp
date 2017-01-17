#include <eeros/safety/SafetySystem.hpp>
#include "ParserTestMainSequence.hpp"
#include "../control/ParserTestControlSystem.hpp"
#include <unistd.h>
#include <iostream>

#include <eeros/hal/Output.hpp>
#include <dlfcn.h>

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
	
	// set PWM frequency here for example or in main of application
	HAL& hal = HAL::instance();	
// 	hal.callOutputFeature(&pwm1, "setPwmFrequency", 100.0);
	
	log.info() << "Starting...";
	for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
		
		if(i%5 == 0){
			log.info() << "enc: " << controlSys->encMot1.getOut().getSignal().getValue();
			log.info() << "analogIn: " << aIn.get();
// 			controlSys->pwm1.getIn().getSignal().setValue(0.4);
// 			pwm1.set(0.2);
// 			log.info() << "seq_enc: " << enc1.get() << std::endl;
		}
		
		if(i%50 == 0){
			log.info() << "reset fqd: ";
			hal.callInputFeature("encMot1", "resetFqd");
		}
		
		if(i%2 == 0){
			controlSys->dac1.getIn().getSignal().setValue(-1);
// 			controlSys->setPos.getOut().getSignal().setValue(true);
			controlSys->io1.getIn().getSignal().setValue(true);
		}
		else{
			controlSys->dac1.getIn().getSignal().setValue(-1);
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