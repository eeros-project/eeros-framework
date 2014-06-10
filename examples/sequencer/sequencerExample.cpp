#include <iostream>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include "MainSequence.hpp"
#include "SequenceA.hpp"
#include "SequenceB.hpp"
#include "SequenceC.hpp"

using namespace eeros::sequencer;
using namespace eeros::logger;

int main(int argc, char* argv[]) {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Sequencer Example started...";
	
	log.info() << "Creating sequences";
	MainSequence mainSequence("Sequence Main");
// 	mainSequence.log.set(w);
	
	SequenceA subSequenceA1("Sequence A1", 0);
// 	subSequenceA1.log.set(w);
	
	SequenceA subSequenceA2("Sequence A2", 100);
// 	subSequenceA2.log.set(w);
	
	SequenceB subSequenceB("Sequence B");
// 	subSequenceB.log.set(w);
	
	SequenceC subSequenceC("Sequence C");
// 	subSequenceC.log.set(w);
	
	log.info() << "Creating and starting sequencer...";
	Sequencer sequencer(mainSequence);
	
	sequencer.join();
	
	log.info() << "Sequencer Example done...";
	
	return 0;
}

