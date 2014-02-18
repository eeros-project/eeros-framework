#include "MainSequence.hpp"
#include <unistd.h>

using namespace eeros::sequencer;
using namespace eeros::logger;

MainSequence::MainSequence(std::string name) : var(0), Sequence(name) {
	log.info() << "Sequencer created: " << name << endl;
}

MainSequence::~MainSequence(){

}

void MainSequence::init() {
	log.info() << "[" << name << "] " << "Init started..." << endl;
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Step  " << var++ << endl;
		sleep(1);
	});
	
	addStep([&]() {
		log.info() << "[" << name << "] " << "Step " << var++ << endl;
		sleep(1);
		for(int i = 0; i < 10; i++) {
			if(i%2 == 0) {
				log.info() << "[" << name << "] " << "Calling sequence A1: " << endl;
				callSubSequence(getSequence("Sequence A1"));
			}
			else {
				log.info() << "[" << name << "] " << "Calling sequence A2: " << endl;
				callSubSequence(getSequence("Sequence A2"));
			}
		}
		sleep(1);
	});
	
	log.info() << "[" << name << "] " << "Init done!" << endl;
}

void MainSequence::exit() {
	log.info() << "[" << name << "] " << "Exit started..." << endl;
	
	log.info() << "[" << name << "] " << "Exit done!" << endl;
}