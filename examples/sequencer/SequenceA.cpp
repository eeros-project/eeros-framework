#include "SequenceA.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, int param) : param(param), Sequence(name) {
	log.info() << "Sequencer created: " << name;
}

SequenceA::~SequenceA(){

}

void SequenceA::init() {
	log.info() << "[" << name << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Param = " << param;
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
	
	log.info() << "[" << name << "] " << "Init done!";
}

void SequenceA::exit() {
	log.info() << "[" << name << "] " << "Exit started...";
	
	log.info() << "[" << name << "] " << "Exit done!";
}

bool SequenceA::checkPreCondition() {
	log.info() << "[" << name << "] " << "Checking precondition...";
	if(param < 100) {
		log.warn() << "[" << name << "] " << "param < 100, adding 1000";
		param += 1000;
	}
	return true;
}

bool SequenceA::checkPostCondition() {
	log.info() << "[" << name << "] " << "Checking postcondition...";
	bool error = param < 100;
	if(error) {
		log.error() << "[" << name << "] " << "param < 100!";
		
	}
	return !error;
}