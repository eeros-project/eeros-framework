#include <iostream>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include "MainSequence.hpp"
#include "SequenceA.hpp"
#include "SequenceB.hpp"

using namespace eeros::sequencer;
using namespace eeros::logger;

int main(int argc, char* argv[]) {
	Logger<LogWriter> log;
	StreamLogWriter w(std::cout);
	log.set(w);
	
	log.info() << "Sequencer Example started..." << endl;
	
	log.info() << "Creating sequences" << endl;
	MainSequence mainSequence("Sequence Main");
	mainSequence.log.set(w);
	
	SequenceA subSequenceA1("Sequence A1", 0);
	subSequenceA1.log.set(w);
	
	SequenceA subSequenceA2("Sequence A2", 100);
	subSequenceA2.log.set(w);
	
	SequenceB subSequenceB("Sequence B");
	subSequenceB.log.set(w);
	
	SequenceB subSequenceC("Sequence C");
	subSequenceC.log.set(w);
	
	log.info() << "Creating and starting sequencer..." << endl;
	Sequencer sequencer("Example sequencer", mainSequence);
	
	while(!sequencer.done());
	
	log.info() << "Sequencer Example done..." << endl;
	
	return 0;
}

