#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

int Sequencer::instanceCounter = 0;

Sequencer::Sequencer(Sequence* startSequence) : id(instanceCounter++), s(notStarted) {
	if(startSequence != nullptr && registerSequence(startSequence)) {
		currentSequence = startSequence;
	}
}

bool Sequencer::registerSequence(Sequence* sequence) {
	if(sequence != nullptr && sequences.insert( {sequence->name, sequence} ).second) {
		sequence->setSequencer(this);
		log.trace() << "Sequencer #" << id << ": Sequence '" << sequence->getName() << "' registered (as #" << static_cast<unsigned int>(sequences.size() - 1) << ").";
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

bool Sequencer::isRegistered(const Sequence* sequence) const {
	for(auto s : sequences) {
		if(s.second == sequence) return true;
	}
	return false;
}

void Sequencer::run() {
	while(s != terminating) {
		if(s != notStarted && currentSequence != nullptr) {
			try {
				currentSequence.load()->run();
				currentSequence = nullptr;
				if(s != stepping) shutdown();
			}
			catch(...) {
				log.warn() << "Uncatched exception, switching to step mode.";
				currentSequence = nullptr;
				stepMode(true);
			}
		}
	}
	s = terminated;
	log.trace() << "Sequencer #" << id << ": has stopped";
}

void Sequencer::start(bool stepMode) {
	if(currentSequence != nullptr) {
		if(stepMode) s = stepping;
		else s = automatic;
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
	s = terminating;
}

void Sequencer::stepMode(bool on) {
	if(on) s = stepping;
	else s = automatic;
}

void Sequencer::yield() {
	status store = s;
	if(s == stepping) {
		std::unique_lock<std::mutex> lck(mtx);
		s = waiting;
		go = false;
		while(!go) cv.wait(lck);
		s = store;
	}
}

void Sequencer::proceed() {
	std::unique_lock<std::mutex> lck(mtx);
	go = true;
	cv.notify_one();
}

void Sequencer::abort() {
	if(currentSequence != nullptr) {
		currentSequence.load()->abort();
	}
}

const std::map<std::string, Sequence*>& Sequencer::getListOfRegisteredSequences() {
	return sequences;
}

Sequencer::status Sequencer::getStatus() const {
	return s;
}
