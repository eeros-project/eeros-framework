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
	log.info() << "[" << name << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Lalala... I'm doing something...";
		sleep(1);
	});
		
	log.info() << "[" << name << "] " << "Init done!";
}

void SequenceC::exit() {
	log.info() << "[" << name << "] " << "Exit started...";
	
	log.info() << "[" << name << "] " << "Exit done!";
}