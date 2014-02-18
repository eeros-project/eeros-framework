#include "SequenceC.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

SequenceC::SequenceC(std::string name) : Sequence(name) {
	log.info() << "Sequencer created: " << name << endl;
}

SequenceC::~SequenceC(){

}

void SequenceC::init() {
	log.info() << "[" << name << "] " << "Init started..." << endl;
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Lalala... I'm doing something..." << endl;
		sleep(1);
	});
		
	log.info() << "[" << name << "] " << "Init done!" << endl;
}

void SequenceC::exit() {
	log.info() << "[" << name << "] " << "Exit started..." << endl;
	
	log.info() << "[" << name << "] " << "Exit done!" << endl;
}