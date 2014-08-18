#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::sequencer;

int Sequencer::instanceCounter = 0;

Sequencer::Sequencer() : id(instanceCounter++), state(state::idle), mode(mode::automatic), currentSequence(nullptr), abortSequence(false) {
// 	if(mainSequence != nullptr) {
// 		currentSequence = mainSequence;
// 	}
}

void Sequencer::run() {
	while(state != state::terminating) {
		if(state != state::idle && currentSequence != nullptr) {
			try {
				(*currentSequence.load())();
				currentSequence = nullptr;
				if(mode == mode::automatic) shutdown();
				else state = state::idle;
			}
			catch(EEROSException& e) {
				log.warn() << "Uncaught exception, switching to step mode." << logger::endl << e.what();
				currentSequence = nullptr;
				stepMode();
				state = state::idle;
			}
			catch(...) {
				log.warn() << "Uncaught exception, switching to step mode.";
				currentSequence = nullptr;
				stepMode();
				state = state::idle;
			}
		}
	}
	state = state::terminated;
	log.trace() << "Sequencer " << getName() << " has stopped";
}

void Sequencer::start() {
	if(currentSequence != nullptr) {
		state = state::executing;
	}
	else {
		log.error() << "Sequencer " << getName() << " failed to start: no sequence specified!";
	}
}

void Sequencer::start(Sequence<>* sequence) {
	if(sequence != nullptr) {
		currentSequence = sequence;
		start();
	}
	else {
		log.error() << "Sequencer " << getName() << " failed to start: sequence is null!";
	}
}

void Sequencer::shutdown() {
	state = state::terminating;
}

void Sequencer::stepMode(bool on) {
	if(on) {
		mode = mode::stepping;
		log.trace() << "Sequencer " << getName() << ": mode set to 'stepping'";
	}
	else {
		mode = mode::automatic;
		log.trace() << "Sequencer " << getName() << ": mode set to 'automatic'";
	}
}

void Sequencer::toggleMode() {
	if(mode == mode::stepping) {
		mode = mode::automatic;
		proceed();
		log.trace() << "Sequencer " << getName() << ": mode toggled to 'automatic'";
	}
	else {
		mode = mode::stepping;
		log.trace() << "Sequencer " << getName() << ": mode toggled to 'stepping'";
	}
}

void Sequencer::yield() {
	if(abortSequence) {
		abortSequence = false;
		throw EEROSException("Sequence aborted");
	}
	else if(mode == mode::stepping) {
		std::unique_lock<std::mutex> lck(mtx);
		state = state::waiting;
		go = false;
		while(!go) cv.wait(lck);
		state = state::executing;
	}
}

void Sequencer::proceed() {
	std::unique_lock<std::mutex> lck(mtx);
	go = true;
	cv.notify_all();
}

void Sequencer::abort() {
	if(currentSequence != nullptr) {
		log.info() << "Sequencer " << getName() << ": stopping execution of sequence '" << currentSequence.load()->getName() << "'";
		abortSequence = true;
		proceed();
	}
}

state::type Sequencer::getState() const {
	return state;
}

mode::type Sequencer::getMode() const {
	return mode;
}

std::string Sequencer::getName() const {
	return std::to_string(id);
}
