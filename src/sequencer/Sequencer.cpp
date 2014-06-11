#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

int Sequencer::instanceCounter = 0;

Sequencer::Sequencer(Sequence* startSequence) : id(instanceCounter++), s(notStarted) {
	if(startSequence != nullptr && registerSequence(startSequence)) {
		setStartSequence(startSequence);
	}
}

bool Sequencer::registerSequence(Sequence* sequence) {
	if(sequence != nullptr && sequences.insert( {sequence->name, sequence} ).second) {
		sequence->setSequencer(this);
		log.trace() << "Sequencer #" << id << ": Sequence '" << sequence->getName() << "' registered.";
		return true;
	}
	else {
		log.error() << "Sequence '" << sequence->name << "' already registered, please choose a unique name!";
	}
	return false;
}

Sequence* Sequencer::getRegisteredSequence(std::string name) {
	return sequences[name];
}

bool Sequencer::isSequenceRegistered(Sequence* sequence) {
	for(auto s : sequences) {
		if(s.second == sequence) return true;
	}
	return false;
}

bool Sequencer::setStartSequence(Sequence* s) {
	if(s != nullptr) {
		startSequence = s;
		log.trace() << "Sequencer #" << id << ": Sequence '" << s->getName() << "' set as start sequence.";
		return true;
	}
	return false;
}

void Sequencer::run() {
	while(s == notStarted);
	try {
		startSequence->run();
	}
	catch(...) {
		log.warn() << "Uncatched exception, switching to step mode.";
		stepMode(true);
	}
}

void Sequencer::start(bool stepMode) {
	if(stepMode) s = stepping;
	else s = running;
}

void Sequencer::stepMode(bool on) {
	if(on) s = stepping;
	else s = running;
}

void Sequencer::yield() {
	if(s == stepping) {
		std::unique_lock<std::mutex> lck(mtx);
		go = false;
		while(!go) cv.wait(lck);
	}
}

void Sequencer::proceed() {
	std::unique_lock<std::mutex> lck(mtx);
	go = true;
	cv.notify_one();
}
