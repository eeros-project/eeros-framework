#include <iostream>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;

class MainSequence : public Sequence {
public:
	MainSequence(std::string name) : Sequence(name), counter(0) {
		log.trace() << "Creating new Sequence '" << name << "'";
		
		addStep([&]() {
			log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
			sleep(1);
			throw -1;
		});
		
		addStep([&]() {
			log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
			sleep(1);
		});
		
		addStep([&]() {
			log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
			sleep(1);
		});
	}
	
	void init() {
		log.trace() << "Sequence '" << getName() << "': " << "Init started";
		sleep(1);
		log.trace() << "Sequence '" << getName() << "': " << "Init done";
	}
	
	void exit() {
		log.trace() << "Sequence '" << getName() << "': " << "Exit started";
		sleep(1);
		log.trace() << "Sequence '" << getName() << "': " << "Exit done";
	}

private:
	int counter;
};

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	MainSequence mainSeq("Main Sequence");
	Sequencer sequencer(&mainSeq);
	sequencer.start();
	
	char k;
	bool stop = false;
	
	while(!stop) {
		std::cout << "Press 'g' for next step or 'e' for exit" << std::endl;
		std::cin >> k;
		switch(k) {
			case 'g':
				sequencer.proceed();
				break;
			case 'e':
				stop = true;
				break;
			default:
				// do nothing
				break;
		}
	}
	sequencer.join();
	
	log.info() << "Martin Test 2 finished...";
}
