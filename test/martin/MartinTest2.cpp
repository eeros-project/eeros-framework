#include <iostream>
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/ui/CursesUI.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;
using namespace eeros::ui;

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
// 			std::cout << "next pointer: " << this->next << std::endl;
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
	SysLogWriter w("martinTest2");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	ExampleSequence subSeqB("Sub Sequence B");
	ExampleSequence subSeqA("Sub Sequence A", &subSeqB);
	ExampleSequence mainSeq("Main Sequence", &subSeqA);
	Sequencer sequencer;
	CursesUI ui(sequencer);
	sequencer.registerSequence(&mainSeq);
	sequencer.registerSequence(&subSeqA);
	sequencer.registerSequence(&subSeqB);
	
// 	log.trace() << "main sequence:  " << reinterpret_cast<long>(&mainSeq);
// 	log.trace() << "sub sequence A: " << reinterpret_cast<long>(&subSeqA);
// 	log.trace() << "sub sequence B: " << reinterpret_cast<long>(&subSeqB);
	
	ui.dispay();
	sequencer.stepMode();
	sequencer.start(&mainSeq);
	
	sequencer.join();
	ui.exit();
	
	log.info() << "Martin Test 2 finished...";
}
