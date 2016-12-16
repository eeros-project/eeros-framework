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
	
	std::cout << "setFrequency: ";
	HAL& hal = HAL::instance();
	
	auto outObj = hal.getOutput("pwm1");
	
	void *handle = dlsym(outObj->getLibHandle(), "setFrequency");
	if(handle == nullptr){
		throw new eeros::EEROSException("could not find method in dynamic library");
		std::cout << "err: " << dlerror() << std::endl;
	}
	
	void (*setFreqPtr)(eeros::hal::OutputInterface*, double) = reinterpret_cast<void(*)(eeros::hal::OutputInterface*, double)>(handle);
	
	(setFreqPtr)(outObj, 100);
	
// 	void *setFrequencyHandle();
// 	hal.getOutputFeature("dac1", "setFrequency", setFrequencyHandle);
// 	setFrequencyHandle(controlSys->pwm1, 100);
	
	
	log.info() << "Starting...";
	for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
		
		if(i%5 == 0){
			std::cout << "enc: " << controlSys->encMot1.getOut().getSignal().getValue() << std::endl;
			controlSys->pwm1.getIn().getSignal().setValue(0.4);
		}
		
		if(i%4 == 0){
			controlSys->dac1.getIn().getSignal().setValue(2.5);
// 			controlSys->setPos.getOut().getSignal().setValue(true);
			controlSys->io1.getIn().getSignal().setValue(true);
		}
		else{
			controlSys->dac1.getIn().getSignal().setValue(-2.5);
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