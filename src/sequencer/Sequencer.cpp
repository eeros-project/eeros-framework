#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

Sequencer::Sequencer(Sequence* startSequence) {
	if(registerSequence(startSequence)) {
		setStartSequence(startSequence);
	}
	else {
		log.error() << "Sequence '" << startSequence->name << "' already registered, please choose a unique name!";
	}
}

Sequencer::Sequencer(Sequence& startSequence) {
	if(registerSequence(&startSequence)) {
		setStartSequence(&startSequence);
	}
	else {
		log.error() << "Sequence '" << startSequence.name << "' already registered, please choose a unique name!";
	}
}

bool Sequencer::registerSequence(Sequence* sequence) {
	if(sequence != nullptr && sequences.insert( {sequence->name, sequence} ).second) {
		sequence->setSequencer(this);
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
	startSequence = s;
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
	std::unique_lock<std::mutex> lck(mtx);
	go = false;
	while(!go) cv.wait(lck);
}

void Sequencer::proceed() {
	std::unique_lock<std::mutex> lck(mtx);
	go = true;
	cv.notify_one();
}
