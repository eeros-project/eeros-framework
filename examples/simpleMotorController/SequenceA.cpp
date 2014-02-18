#include "SequenceA.hpp"
#include "MyControlSystem.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
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
		sleep(1);
		controlSys.setpoint.setValue(angle);
	});
	
	log.info() << "[" << name << "] " << "Init done!";
}

void SequenceA::exit() {
	log.info() << "[" << name << "] " << "Exit started...";
	
	log.info() << "[" << name << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
	log.info() << "[" << name << "] " << "Checking precondition...";
	return true;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << name << "] " << "Checking postcondition...";
	return true;
}