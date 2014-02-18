#include <eeros/sequencer/Sequence.hpp>

using namespace eeros::sequencer;
using namespace eeros::logger;

std::map<std::string, Sequence*> Sequence::allSequences;

Sequence::Sequence(std::string name) : name(name) {
	// reset sequence
	reset();
	
	// add the sequence to the list with all sequences
	auto res = Sequence::allSequences.insert( {name, this} ).second;
	if(res == false) {
		throw -1; // TODO
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
				// TODO counter
				break;
			default:
				throw -454; // TODO
				break;
		}
		
		while(!checkPreCondition());
		
		// execute action lambdas
		while(currentStep < actionList.size()) {
			actionList[currentStep]();
			currentStep++;
		}
		
// 		for(auto action : actionList) {
// 			action();
// 			currentStep++;
// 		}

		// check postcondition
		if(!checkPostCondition()) throw -715; // TODO
		
		// cleanup and exit
		exit();
        
	}
	catch(SequenceException const &e) {
		log.info() << e.what();
		e.handle();
		returnBehavior = e.getReturnBehavior();
		
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
				throw -789; // TODO
				break;
		}
	}
	// TODO catch(EEROSException)
	// TODO catch(std::exception)
}

void Sequence::init() {
    // TODO
}

bool Sequence::checkPreCondition() {
    return true;
}

bool Sequence::checkPostCondition() {
    return true;
}

void Sequence::exit() {
    // TODO
}


void Sequence::reset() {
	state = kSequenceNotStarted;
	returnBehavior = kContinue;
	currentStep = 0;
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