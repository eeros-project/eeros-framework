#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;

Sequence::Sequence(std::string name, Sequencer* sequencer) : name(name), sequencer(sequencer), abortRequest(false) {
	if(sequencer != nullptr) {
		sequencer->registerSequence(this);
	}
	reset(); // reset sequence
}

std::string Sequence::getName() const {
	return name;
}

int Sequence::getState() const {
	return state;
}

void Sequence::run() {
	try {
		switch(state) {
			case kSequenceFinished:
				reset();
				init();
				break;
			case kSequenceNotStarted:
				init();
				break;
			case kSequenceException:
				exceptionRetryCounter++;
				break;
			default:
				log.error() << "Illegal sequence state '" << (int)state << "', resetting sequence!";
				reset();
				init();
				break;
		}
		yield();
		
		while(!checkPreCondition()) {
			yield();
		};
		
		// execute action lambdas
		while(currentStep < steps.size()) {
			yield();
			steps[currentStep]();
			currentStep++;
		}
		
		// check postcondition
		yield();
		if(!checkPostCondition()) throw -715; // TODO
		
		// cleanup and exit
		exit();
	}
	catch(SequenceException const &e) {
		log.info() << e.what();
		e.handle();
		ExceptionReturnBehavior returnBehavior = e.getReturnBehavior();
		
		switch(returnBehavior) {
			case kRepeatSequence:
				reset();
				break;
			case kRepeatStep:
				this->run();
				break;
			case kContinue:
				currentStep++;
				this->run();
				break;
			case kNewSequence:
				state = kSequenceFinished;
				call(e.getEnsuingSequence());
				break;
			default:
				log.error() << "Illegal return behavior '" << (int)returnBehavior << "' in exception '" << e.what() << "'!";
				break;
		}
	}
}

void Sequence::init() {
	// nothing to do
}

bool Sequence::checkPreCondition() {
	return true;
}

bool Sequence::checkPostCondition() {
	return true;
}

void Sequence::exit() {
	// nothing to do
}

void Sequence::reset() {
	state = kSequenceNotStarted;
	currentStep = 0;
	exceptionRetryCounter = 0;
	abortRequest = false;
}

void Sequence::call(Sequence* sequence) {
	if(sequence != nullptr) {
		if(sequencer->isRegistered(sequence)) {
			log.trace() << "Call to sequence '" << sequence->getName() << "'.";
			sequence->run();
		}
		else {
			log.warn() << "Call to unregistered sequence ignored!" << endl;
		}
	}
	else {
		log.warn() << "Call to NULL sequence ignored!" << endl;
	}
}

void Sequence::call(std::string sequenceName) {
	call(sequencer->getRegisteredSequence(sequenceName));
}

// void Sequence::start(Sequence* sequence) {
// 	if(sequence != nullptr) {
// 		new Sequencer("parallelSequencer", *sequence); // TODO improve this!
// 	}
// 	else {
// 		log.warn() << "Parallel start of NULL sequence ignored!" << endl;
// 	}
// }

void Sequence::addStep(std::function<void(void)> action) {
	steps.push_back(action);
}

void Sequence::yield() {
	if(sequencer != nullptr) {
		sequencer->yield();
	}
	if(abortRequest.load() == true) {
		throw EEROSException("Sequence aborted by user");
	}
}

void Sequence::setSequencer(Sequencer* sequencer) {
	this->sequencer = sequencer;
}

void Sequence::abort() {
	abortRequest = true;
}
