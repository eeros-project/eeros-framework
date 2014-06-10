#include "SequenceC.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceC::SequenceC(std::string name) : Sequence(name) {
	log.info() << "Sequence created: " << name;
}

SequenceC::~SequenceC(){

}

void SequenceC::init() {
	log.info() << "[" << getName() << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << getName() << "] " << "Lalala... I'm doing something...";
		sleep(1);
	});
		
	log.info() << "[" << getName() << "] " << "Init done!";
}

void SequenceC::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	
	log.info() << "[" << getName() << "] " << "Exit done!";
}
