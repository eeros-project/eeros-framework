#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

int Sequencer::instanceCounter = 0;

Sequencer::Sequencer(Sequence* startSequence) : id(instanceCounter++), s(waiting) {
	if(startSequence != nullptr && registerSequence(startSequence)) {
		currentSequence = startSequence;
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

bool Sequencer::isRegistered(const Sequence* sequence) {
	for(auto s : sequences) {
		if(s.second == sequence) return true;
	}
	return false;
}

void Sequencer::run() {
	while(s != stopping) {
		if(s != waiting && currentSequence != nullptr) {
			try {
				currentSequence.load()->run();
				currentSequence = nullptr;
			}
			catch(...) {
				log.warn() << "Uncatched exception, switching to step mode.";
				currentSequence = nullptr;
				stepMode(true);
			}
		}
	}
	s = stopped;
	log.trace() << "Sequencer #" << id << ": has stopped";
}

void Sequencer::start(bool stepMode) {
	if(currentSequence != nullptr) {
		if(stepMode) s = stepping;
		else s = running;
	}
	else {
		log.error() << "Sequencer #" << id << ": failed to start, no sequence specified!";
	}
}

void Sequencer::start(Sequence* sequence, bool stepMode) {
	if(isRegistered(sequence)) {
		currentSequence = sequence;
		start(stepMode);
	}
	else {
		log.error() << "Sequencer #" << id << ": failed to start, sequence not registered!";
	}
}

void Sequencer::shutdown() {
	s = stopping;
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
