#include "SequenceB.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceB::SequenceB(std::string name) : Sequence(name) {
	log.info() << "Sequence created: " << name;
}

SequenceB::~SequenceB(){

}

void SequenceB::init() {
	log.info() << "[" << getName() << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << getName() << "] " << "First step...";
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << getName() << "] " << "Second step...";
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << getName() << "] " << "Third step...";
		sleep(1);
	});
	
	log.info() << "[" << getName() << "] " << "Init done!";
}

void SequenceB::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	
	log.info() << "[" << getName() << "] " << "Exit done!";
}
