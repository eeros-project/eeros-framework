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

class DetectSequence : public Sequence<bool, int> {
public:
	DetectSequence(std::string name, Sequencer* sequencer) : Sequence<bool, int>(name, sequencer) { }
		
	virtual bool checkPreCondition() {
		return false;
	}
		
	bool run(int x) {
		log.trace() << "Sequence '" << getName() << "': x = " << x;
		sleep(1);
		return x % 2;
	}
};

class MainSequence : public Sequence<> {
public:
	MainSequence(std::string name, Sequencer* sequencer) : Sequence<>(name, sequencer), counter(0), detect("detect", sequencer) {
		log.trace() << "Creating new sequence '" << name << "'...";
	}
		
	void run() {
		log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
		sleep(1);
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
		sleep(1);
		auto res = detect(counter + 1);
		log.trace() << "Sequence '" << getName() << "': calling subsequence: " << res.value << "/" << res.result;
		yield();
		
		log.trace() << "Sequence '" << getName() << "': " << "Step " << counter++;
		sleep(1);
		yield();
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
	Sequence<>* next;
	DetectSequence detect;
};


int main() {
	// Create and initialize logger
	SysLogWriter w("martinTest2");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	Sequencer sequencer;
	CursesUI ui(sequencer);
	ui.dispay();
	
	MainSequence mainSeq("Main Sequence", &sequencer);
	
	sequencer.stepMode();
	sequencer.start(&mainSeq);
	
	sequencer.join();

	log.info() << "Martin Test 2 finished...";
}
