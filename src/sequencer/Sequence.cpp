#include <eeros/sequencer/Sequence.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::sequencer;
using namespace eeros::logger;

std::map<std::string, Sequence*> Sequence::allSequences;

Sequence::Sequence(std::string name) : name(name) {
	// reset sequence
	reset();
	
	// add the sequence to the list with all sequences
	if(!Sequence::allSequences.insert( {name, this} ).second) {
		log.error() << "Sequence '" << name << "' already exists, please choose a unique name!";
	}
}

Sequence::~Sequence() {
	// remove the sequence from the list with all sequeces
	Sequence::allSequences.erase(name);
}

std::string Sequence::getName(){
	return name;
}

int Sequence::getState() {
    return state;
}

void Sequence::run() {
	try{
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
		
		while(!checkPreCondition()); // TODO add abort condition
		
		// execute action lambdas
		while(currentStep < actionList.size()) {
			actionList[currentStep]();
			currentStep++;
		}
		
		// check postcondition
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
				callSubSequence(e.getEnsuingSequence());
				break;
			default:
				log.error() << "Illegal return behavior '" << (int)returnBehavior << "' in exception '" << e.what() << "'!";
				break;
		}
	}
	// TODO catch(EEROSException)
	// TODO catch(std::exception)
}

void Sequence::init() {
    
}

bool Sequence::checkPreCondition() {
    return true;
}

bool Sequence::checkPostCondition() {
    return true;
}

void Sequence::exit() {
    
}

void Sequence::reset() {
	state = kSequenceNotStarted;
	currentStep = 0;
	exceptionRetryCounter = 0;
}

void Sequence::callSubSequence(Sequence* sequence) {
	if(sequence != nullptr) {
		sequence->run();
	}
	else {
		log.warn() << "Call to NULL sequence ignored!" << endl;
	}
}

void Sequence::startParallelSequence(Sequence* sequence) {
	// TODO
}

void Sequence::addStep(std::function<void(void)> action) {
	actionList.push_back(action);
}

Sequence* Sequence::getSequence(std::string name){
	return Sequence::allSequences[name];
}