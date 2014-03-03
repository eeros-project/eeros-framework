#include "SequenceA.hpp"
#include "MyControlSystem.hpp"
#include "MySafetyProperties.hpp"
#include <eeros/safety/SafetySystem.hpp>
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, double angle) : angle(angle), Sequence(name) {
	log.info() << "Sequence created: " << name;
}

SequenceA::~SequenceA(){

}

void SequenceA::init() {
	log.info() << "[" << name << "] " << "Init started...";
	
	MyControlSystem& controlSys = MyControlSystem::instance();
	
	addStep([&]() {
		double a = 0;
		sleep(5);
		for(int i = 0; i < 10; i++) {
			log.info() << "[" << name << "] " << "setting angle to " << a;
			controlSys.setpoint.setValue(a);
			a += angle;
			sleep(1);
			log.info() << "[" << name << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
			sleep(1);
		}
		
	});
	
	addStep([&]() {
		sleep(1);
		log.info() << "[" << name << "] " << "setting angle to " << -3.14;
		controlSys.setpoint.setValue(-3.14);
		sleep(1);
		log.info() << "[" << name << "] " << "enc =  " << controlSys.enc.getOut().getSignal().getValue();
		sleep(1);
	});
	
	log.info() << "[" << name << "] " << "Init done!";
}

void SequenceA::exit() {
	log.info() << "[" << name << "] " << "Exit started...";
	
	log.info() << "[" << name << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
//	log.info() << "[" << name << "] " << "Checking precondition...";
	SafetySystem& safetySys = SafetySystem::instance();
	
	return safetySys.getCurrentLevel().getId() >= moving;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << name << "] " << "Checking postcondition...";
	return true;
}