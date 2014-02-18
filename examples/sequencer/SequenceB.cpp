#include "SequenceB.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceB::SequenceB(std::string name) : Sequence(name) {
	log.info() << "Sequencer created: " << name << endl;
}

SequenceB::~SequenceB(){

}

void SequenceB::init() {
	log.info() << "[" << name << "] " << "Init started..." << endl;
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "First step..." << endl;
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Second step..." << endl;
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Third step..." << endl;
		sleep(1);
	});
	
	log.info() << "[" << name << "] " << "Init done!" << endl;
}

void SequenceB::exit() {
	log.info() << "[" << name << "] " << "Exit started..." << endl;
	
	log.info() << "[" << name << "] " << "Exit done!" << endl;
}