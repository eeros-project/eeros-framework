#include "SequenceA.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, int param) : param(param), Sequence(name) {
	log.info() << "Sequence created: " << name;
}

SequenceA::~SequenceA(){

}

void SequenceA::init() {
	log.info() << "[" << getName() << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << getName() << "] " << "Param = " << param;
		switch(param) {
			case 101:
				param++;
				throw SequenceException("Exception: param == 101", getSequence("Sequence C"), kContinue);
				break;
			case 103:
				param++;
				throw SequenceException("Exception: param == 103", getSequence("Sequence C"), kRepeatStep);
				break;
			case 105:
				throw SequenceException("Exception: param == 105", getSequence("Sequence C"), kNewSequence, getSequence("Sequence B"));
				break;
			default:
				// do nothing
				break;
		}
		param++;
		sleep(1);
	});
	
	log.info() << "[" << getName() << "] " << "Init done!";
}

void SequenceA::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	
	log.info() << "[" << getName() << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
	log.info() << "[" << getName() << "] " << "Checking precondition...";
	if(param < 100) {
		log.warn() << "[" << getName() << "] " << "param < 100, adding 1000";
		param += 1000;
	}
	return true;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << getName() << "] " << "Checking postcondition...";
	bool error = param < 100;
	if(error) {
		log.error() << "[" << getName() << "] " << "param < 100!";
		
	}
	return !error;
}
