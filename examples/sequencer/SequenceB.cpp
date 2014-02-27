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
	log.info() << "[" << name << "] " << "Init started...";
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "First step...";
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Second step...";
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Third step...";
		sleep(1);
	});
	
	log.info() << "[" << name << "] " << "Init done!";
}

void SequenceB::exit() {
	log.info() << "[" << name << "] " << "Exit started...";
	
	log.info() << "[" << name << "] " << "Exit done!";
}