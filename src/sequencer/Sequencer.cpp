#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

Sequence& Sequencer::getMainSequence(){
	return mainSequence;
}

Sequencer::Sequencer(std::string name, Sequence& mainSequence) : mainSequence(mainSequence), sequenceExecutor(0), timeoutExecutor(timeoutExecutorPeriod), name(name) {
	sequenceExecutor.addRunnable(mainSequence);
	sequenceExecutor.start();
}

Sequencer::~Sequencer(){	
	sequenceExecutor.stop();
}

std::string Sequencer::getName(){
	return name;
}

bool Sequencer::done(){
	return sequenceExecutor.isTerminated();
}