#include <iostream>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;

class ExampleSequence : public Sequence {
public:
	ExampleSequence(std::string name, Sequence* nextSequence = nullptr) : Sequence(name), counter(0), next(nextSequence) {
		log.trace() << "Creating new sequence '" << name << "'...";
		
		addStep([&]() {
			log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
			sleep(1);
		});
		
		addStep([&]() {
			log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
			sleep(1);
			std::cout << "next pointer: " << this->next << std::endl;
			if(this->next != nullptr) call(this->next);
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
	Sequence* next;
};

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	ExampleSequence subSeqB("Sub Sequence B");
	ExampleSequence subSeqA("Sub Sequence A", &subSeqB);
	ExampleSequence mainSeq("Main Sequence", &subSeqA);
	Sequencer sequencer;
	sequencer.registerSequence(&mainSeq);
	sequencer.registerSequence(&subSeqA);
	sequencer.registerSequence(&subSeqB);
	
	std::cout << "main sequence:  " << &mainSeq << std::endl;
	std::cout << "sub sequence A: " << &subSeqA << std::endl;
	std::cout << "sub sequence B: " << &subSeqB << std::endl;
	
	sequencer.start(&mainSeq);
	sequencer.stepMode(true);
	
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
				std::cout << "Stopping sequencer and exiting..." << std::endl;
				sequencer.shutdown();
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
