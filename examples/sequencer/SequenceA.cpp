#include "SequenceA.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceA::SequenceA(std::string name, int param) : param(param), Sequence(name) {
	log.info() << "Sequencer created: " << name << endl;
}

SequenceA::~SequenceA(){

}

void SequenceA::init() {
	log.info() << "[" << name << "] " << "Init started..." << endl;
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Param = " << param << endl;
		switch(param) {
			case 101:
				param++;
				throw SequenceException("Exception: param == 101", getSequence("Sequence C"), kContinue, NULL);
				break;
			case 103:
				param++;
				throw SequenceException("Exception: param == 103", getSequence("Sequence C"), kRepeatStep, NULL);
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
	
	log.info() << "[" << name << "] " << "Init done!" << endl;
}

void SequenceA::exit() {
	log.info() << "[" << name << "] " << "Exit started..." << endl;
	
	log.info() << "[" << name << "] " << "Exit done!" << endl;
}