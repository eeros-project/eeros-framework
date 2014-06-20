#include <eeros/sequencer/Sequencer.hpp>

using namespace eeros;
using namespace eeros::sequencer;

int Sequencer::instanceCounter = 0;

Sequencer::Sequencer(Sequence* startSequence) : id(instanceCounter++), state(idle), mode(automatic) {
	if(startSequence != nullptr && registerSequence(startSequence)) {
		currentSequence = startSequence;
	}
}

bool Sequencer::registerSequence(Sequence* sequence) {
	if(sequence != nullptr && sequences.insert( {sequence->name, sequence} ).second) {
		sequence->setSequencer(this);
		log.trace() << "Sequencer " << getName() << ": Sequence '" << sequence->getName() << "' registered (as #" << static_cast<unsigned int>(sequences.size() - 1) << ").";
		return true;
	}
	else {
		log.error() << "Sequencer " << getName() << ": Sequence '" << sequence->name << "' already registered, please choose a unique name!";
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
	while(state != terminating) {
		if(state != idle && currentSequence != nullptr) {
			try {
				currentSequence.load()->run();
				currentSequence = nullptr;
				if(mode == automatic) shutdown();
				else state = idle;
			}
			catch(...) {
				log.warn() << "Uncatched exception, switching to step mode.";
				currentSequence = nullptr;
				stepMode();
				state = idle;
			}
		}
	}
	state = terminated;
	log.trace() << "Sequencer " << getName() << " has stopped";
}

void Sequencer::start() {
	if(currentSequence != nullptr) {
		state = executing;
	}
	else {
		log.error() << "Sequencer " << getName() << " failed to start: no sequence specified!";
	}
}

void Sequencer::start(Sequence* sequence) {
	if(isRegistered(sequence)) {
		currentSequence = sequence;
		start();
	}
	else {
		log.error() << "Sequencer " << getName() << " failed to start: sequence not registered!";
	}
}

void Sequencer::shutdown() {
	state = terminating;
}

void Sequencer::stepMode(bool on) {
	if(on) {
		mode = stepping;
		log.trace() << "Sequencer " << getName() << ": mode set to 'stepping'";
	}
	else {
		mode = automatic;
		log.trace() << "Sequencer " << getName() << ": mode set to 'automatic'";
	}
}

void Sequencer::toggleMode() {
	if(mode == stepping) {
		mode == automatic;
		log.trace() << "Sequencer " << getName() << ": mode toggled to 'automatic'";
	}
	else {
		mode == stepping;
		log.trace() << "Sequencer " << getName() << ": mode toggled to 'stepping'";
	}
}

void Sequencer::yield() {
	if(mode == stepping) {
		std::unique_lock<std::mutex> lck(mtx);
		state = waiting;
		go = false;
		while(!go) cv.wait(lck);
		state = executing;
	}
}

void Sequencer::proceed() {
	std::unique_lock<std::mutex> lck(mtx);
	go = true;
	cv.notify_one();
}

void Sequencer::abort() {
	if(currentSequence != nullptr) {
		log.info() << "Sequencer " << getName() << ": stopping execution of sequence '" << currentSequence.load()->getName() << "'";
		currentSequence.load()->abort();
		proceed();
	}
}

const std::map<std::string, Sequence*>& Sequencer::getListOfRegisteredSequences() {
	return sequences;
}

Sequencer::State Sequencer::getState() const {
	return state;
}

Sequencer::Mode Sequencer::getMode() const {
	return mode;
}


std::string Sequencer::getName() const {
	return std::to_string(id);
}

const Sequence* Sequencer::getCurrentSequence() const {
	return currentSequence;
}
