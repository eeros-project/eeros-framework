#include "MainSequence.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

MainSequence::MainSequence(std::string name) : var(0), Sequence(name) {
	log.info() << "Sequence created: " << name;
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Step  " << var++;
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Step " << var++;
		sleep(1);
		for(int i = 0; i < 10; i++) {
			if(i%2 == 0) {
				log.info() << "[" << name << "] " << "Calling sequence A1: ";
				call(getSequence("Sequence A1"));
			}
			else {
				log.info() << "[" << name << "] " << "Calling sequence A2: ";
				call(getSequence("Sequence A2"));
			}
		}
		sleep(1);
	});
}

MainSequence::~MainSequence(){

}

void MainSequence::init() {
	log.info() << "[" << getName() << "] " << "Init started...";
	
	log.info() << "[" << getName() << "] " << "Init done!";
}

void MainSequence::exit() {
	log.info() << "[" << getName() << "] " << "Exit started...";
	
	log.info() << "[" << getName() << "] " << "Exit done!";
}
