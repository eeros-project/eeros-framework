#include "SequenceA.hpp"
#include "MyControlSystem.hpp"
#include "MySafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, SafetySystem& safetySys, MyControlSystem& controlSys, double angle) : safetySys(safetySys), angle(angle), Sequence(name), controlSys(controlSys) {
	log.info() << "Sequence created: " << name;
}

void SequenceA::init() {
	log.info() << "[" << getName() << "] " << "Init started...";
	
	addStep([&]() {
		double a = 0;
		sleep(5);
		for(int i = 0; i < 10; i++) {
			log.info() << "[" << getName() << "] " << "setting angle to " << a;
			controlSys.setpoint.setValue(a);
			a += angle;
			sleep(1);
			log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
		
	});
	
	addStep([&]() {
		sleep(1);
		log.info() << "[" << getName() << "] " << "setting angle to " << -3.14;
		controlSys.setpoint.setValue(-3.14);
		sleep(1);
		log.info() << "[" << getName() << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
		sleep(1);
	});
	
	log.info() << "[" << getName() << "] " << "Init done!";
}

void SequenceA::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	
	log.info() << "[" << getName() << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
	return safetySys.getCurrentLevel().getId() >= moving;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << getName() << "] " << "Checking postcondition...";
	return true;
}
