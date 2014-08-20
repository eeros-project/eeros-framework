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

int main() {
	// Create and initialize logger
	SysLogWriter w("martinTest2");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	w.show(~0);
	
	log.info() << "Martin Test 2 started...";
	
	// moved to examples

	log.info() << "Martin Test 2 finished...";
}
