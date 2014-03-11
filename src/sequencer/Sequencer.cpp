#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

std::map<std::string, Sequencer*> Sequencer::allSequencers;

Sequence& Sequencer::getMainSequence(){
	return mainSequence;
}

Sequencer::Sequencer(std::string name, Sequence& mainSequence) : mainSequence(mainSequence), sequenceExecutor(0), timeoutExecutor(timeoutExecutorPeriod), name(name) {
	sequenceExecutor.addRunnable(mainSequence);
	sequenceExecutor.start();
	
	// add the sequencer to the list with all sequencers
	Sequencer::allSequencers.insert( {name, this} );
}

Sequencer::~Sequencer() {
	sequenceExecutor.stop();
}

std::string Sequencer::getName(){
	return name;
}

bool Sequencer::done(){
	return sequenceExecutor.isTerminated();
}

Sequencer* Sequencer::getSequencer(std::string name){
	return Sequencer::allSequencers[name];
}
