#include <eeros/sequencer/BaseSequence.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;

BaseSequence::BaseSequence(Sequencer& seq, BaseSequence* caller) : 
	seq(seq), caller(caller), conditionTimeout(), monitorTimeout(this, &conditionTimeout, SequenceProp::abortOwner),
	state(SequenceState::idle), blocking(true), pollingTime(100), log('X')
{
	if (caller == nullptr) {
		static int numberOfMainSequence;
		numberOfMainSequence++;
		if (numberOfMainSequence > 1) 
			throw Fault("Only one main sequence is possible. Use 'Sequence(S, caller, name)' to construct a normal sequence");
		else 
			isMainSequence = true;
	}
	
	//get and update callerStack
	if (!isMainSequence) {
		callerStack = caller->getCallerStack();
		callerStack.push_back(caller);	// add latest caller
	}
	
	//callerStackBlocking gets created, when getCallerStackBlocking() is called
	addMonitor( &monitorTimeout );
}

BaseSequence::~BaseSequence() { }

int BaseSequence::action() {
	state = SequenceState::running;
	// check if this sequence or a blocked caller sequence got the order to abort, restart... , if so, set runningState accordingly
	checkActiveException();
	
	if ((caller != NULL) && (state == SequenceState::running) && (caller->state == SequenceState::restarting)) 
		state = SequenceState::aborting;
	
	do {	//for restarting
		if (state == SequenceState::restarting) {	// sequence got restarted
			state = SequenceState::running;
			sequenceIsRestarting = false;
			resetTimeout();
			restartCounter++;
		}
		else restartCounter = 0;
		
// 		if( runningState == paused ) {	//has to be alerted by an other sequence
// 			usleep(pollingTime*1000);
// 		}
		
		bool firstCheck = true;
		if (checkPreCondition()) {
			action();	// call to custom implementation of method
			while (state == SequenceState::running) {
				if (!firstCheck) {
					if (checkExitCondition()) state = SequenceState::terminated;	// check exit condition
					if (state != SequenceState::terminated) {
// 						log.fatal();
						checkMonitorsOfThisSequence();		//sets activeException if needed
						checkMonitorsOfBlockedCallers();	//sets activeException if needed
					}
				} else firstCheck = false;
				
				checkActiveException();					//sets RunningState according to activeException
				if (state == SequenceState::running) usleep(pollingTime*1000);
			}
		}
		else {	// checkPreCondition() failed
			state = SequenceState::terminated;
		}
	
	} while (state == SequenceState::restarting);
	
	if (state == SequenceState::aborting) state = SequenceState::aborted;
	else state = SequenceState::terminated;
}

void BaseSequence::addMonitor(Monitor* monitor) {monitors.push_back(monitor);}

std::vector< Monitor* > BaseSequence::getMonitors() const {return monitors;}

void BaseSequence::checkMonitorsOfThisSequence() {
// 	log.fatal() << "check mon of this seq";

	for (Monitor* monitor : getMonitors()) checkMonitor(monitor);
}

void BaseSequence::checkMonitorsOfBlockedCallers() {
	// when first called, caller stack has to be created
	if (!callerStackBlockingCreated) {
		std::vector<BaseSequence*> tempStack;
		for (int i = callerStack.size(); i--;) {	// reverse iteration
			if (callerStack[i]->isBlocking()) tempStack.push_back(callerStack[i]);
			else break;
		}
		
		for (int i = tempStack.size(); i--;) {		// reverse vector
			callerStackBlocking.push_back(tempStack[i]);
// 			log.info() << "___callerStackBlocking of seq " << tempStack[i]->getName();
		}
		callerStackBlockingCreated = true;
	}
// 	log.fatal() << "check mon of blocked caller";

	for (BaseSequence* seq : callerStackBlocking) {
		for (Monitor* monitor : seq->getMonitors()) checkMonitor(monitor);
	}
}

void BaseSequence::checkMonitor(Monitor* monitor) {
	if (monitor->checkCondition() == true) {
		monitor->startExceptionSequence();
// 		log.fatal() << "cond true of mon of " << name << " owner=" << monitor->getOwner()->name;
		switch (monitor->getBehavior()) {
			case SequenceProp::nothing : break;
			case SequenceProp::abortOwner : monitor->getOwner()->setActiveException(monitor); break;
			case SequenceProp::restartOwner : monitor->getOwner()->setActiveException(monitor); break;
			case SequenceProp::abortCallerofOwner :	monitor->getOwner()->getCallerSequence()->setActiveException(monitor); break;
			case SequenceProp::restartCallerOfOwner : monitor->getOwner()->getCallerSequence()->setActiveException(monitor); break;
			default : break;
		}
	}
	else return;
}

void BaseSequence::setActiveException(Monitor* activeMonitor) {
	switch (activeMonitor->getBehavior()) {
		case SequenceProp::abortOwner :
		case SequenceProp::restartOwner :
			activeMonitor->getOwner()->exceptionIsActive = true;
			activeMonitor->getOwner()->activeException = activeMonitor;
			break;
		case SequenceProp::abortCallerofOwner :
		case SequenceProp::restartCallerOfOwner :	
			activeMonitor->getOwner()->getCallerSequence()->exceptionIsActive = true;
			activeMonitor->getOwner()->getCallerSequence()->activeException = activeMonitor;
			break;
		default : break;
	}
}

void BaseSequence::clearActiveException() {
	exceptionIsActive = false;
	activeException = nullptr;
}

void BaseSequence::checkActiveException() {
	if (exceptionIsActive == true) {	// this sequence got the order to abort, restart ...
		switch (activeException->getBehavior()) {
			case SequenceProp::nothing : break;
			case SequenceProp::abortOwner : state = SequenceState::aborting; break;
			case SequenceProp::restartOwner : state = SequenceState::restarting; break;
			case SequenceProp::abortCallerofOwner : state = SequenceState::aborting; break;
			case SequenceProp::restartCallerOfOwner : state = SequenceState::restarting; break;
			default : break;
		}
		clearActiveException();
	} else {				// a blocked caller got the order to abort, restart ...
		for (BaseSequence* seq : getCallerStack()) {
			if (seq->exceptionIsActive) {
				switch (seq->activeException->getBehavior()) {
					case SequenceProp::abortOwner :
					case SequenceProp::restartOwner :
					case SequenceProp::abortCallerofOwner :
					case SequenceProp::restartCallerOfOwner : state = SequenceState::aborting; break;
					default : break;
				}
			}
		}
	}
}

// bool BaseSequence::isStep() const {return false;}

bool BaseSequence::checkExitCondition() {return true;}

bool BaseSequence::checkPreCondition() {return true;}

void BaseSequence::setName(std::string name) {this->name = name;}

std::string BaseSequence::getName() const {return name;}

SequenceState BaseSequence::getRunningState() const {return state;}

void BaseSequence::setId(int id) {this->id = id;}

int BaseSequence::getId() const {return id;}

bool BaseSequence::isBlocking() const {return blocking;}

BaseSequence* BaseSequence::getCallerSequence() {
	if (caller == nullptr) {
		log.error() << "This sequence does not have a caller";
		return nullptr;
	}
	else return caller;
}

std::vector<BaseSequence*> BaseSequence::getCallerStack() const {return callerStack;}

void BaseSequence::restartSequence() {
	state = SequenceState::restarting;
	sequenceIsRestarting = true;
}

void BaseSequence::setPollingTime(int timeInMilliseconds) {
	pollingTime = timeInMilliseconds;
}

void BaseSequence::setTimeoutTime(double timeoutInSec) {
	conditionTimeout.setTimeoutTime(timeoutInSec);
}

void BaseSequence::resetTimeout() {
	conditionTimeout.resetTimeout();
}

void BaseSequence::setTimeoutBehavior(SequenceProp behavior) {
	monitorTimeout.setBehavior(behavior);
}

void BaseSequence::setTimeoutExceptionSequence(BaseSequence* sequence) {
	monitorTimeout.setExceptionSequence(sequence);
}