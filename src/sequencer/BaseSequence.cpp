#include <eeros/sequencer/BaseSequence.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;

BaseSequence::BaseSequence(Sequencer& seq, BaseSequence* caller) : 
	seq(seq), callerSequence(caller), conditionTimeout(), monitorTimeout(this, &conditionTimeout, SequenceProp::abortOwner),
	runningState(SequenceState::idle), blocking(true), pollingTime(100), log('X')
{
	if (callerSequence == nullptr) {
		static int numberOfMainSequence;
		numberOfMainSequence++;
		if (numberOfMainSequence > 1) 
			throw Fault("Only one main sequence is possible. Use 'Sequence(S, caller, name)' to construct a normal sequence");
		else 
			isMainSequence = true;
	}
	
	//get and update callerStack
	if (!isMainSequence) {
		callerStack = callerSequence->getCallerStack();
		callerStack.push_back(callerSequence);	// add latest caller
	}
	
	//callerStackBlocking gets created, when getCallerStackBlocking() is called
	addMonitor( &monitorTimeout );
}

BaseSequence::~BaseSequence() { }

int BaseSequence::action() {
	runningState = SequenceState::running;
	// check if this sequence or a blocked caller sequence got the order to abort, restart... , if so, set runningState accordingly
	checkActiveException();
	
	if ((callerSequence != NULL) && (runningState == SequenceState::running) && (callerSequence->runningState == SequenceState::restarting)) 
		runningState = SequenceState::aborting;
	
	do {	//for restarting
		if (runningState == SequenceState::restarting) {	// sequence got restarted
			runningState = SequenceState::running;
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
			while (runningState == SequenceState::running) {
				if (!firstCheck) {
					if (checkExitCondition()) runningState = SequenceState::terminated;	// check exit condition
					if (runningState != SequenceState::terminated) {
// 						log.fatal();
						checkMonitorsOfThisSequence();		//sets activeException if needed
						checkMonitorsOfBlockedCallers();	//sets activeException if needed
					}
				} else firstCheck = false;
				
				checkActiveException();					//sets RunningState according to activeException
				if (runningState == SequenceState::running) usleep(pollingTime*1000);
			}
		}
		else {	// checkPreCondition() failed
			runningState = SequenceState::terminated;
		}
	
	} while (runningState == SequenceState::restarting);
	
	if (runningState == SequenceState::aborting) runningState = SequenceState::aborted;
	else runningState = SequenceState::terminated;
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
			case SequenceProp::abortOwner : runningState = SequenceState::aborting; break;
			case SequenceProp::restartOwner : runningState = SequenceState::restarting; break;
			case SequenceProp::abortCallerofOwner : runningState = SequenceState::aborting; break;
			case SequenceProp::restartCallerOfOwner : runningState = SequenceState::restarting; break;
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
					case SequenceProp::restartCallerOfOwner : runningState = SequenceState::aborting; break;
					default : break;
				}
			}
		}
	}
}

bool BaseSequence::isStep() const {return false;}

bool BaseSequence::checkExitCondition() {return true;}

bool BaseSequence::checkPreCondition() {return true;}

void BaseSequence::setName(std::string name) {this->name = name;}

std::string BaseSequence::getName() const {return name;}

SequenceState BaseSequence::getRunningState() const {return runningState;}

void BaseSequence::setId(int id) {this->id = id;}

int BaseSequence::getId() const {return id;}

void BaseSequence::setBlocking() {blocking = true;}

void BaseSequence::setNonBlocking() {blocking = false;}

bool BaseSequence::isBlocking() const {return blocking;}

BaseSequence* BaseSequence::getCallerSequence() {
	if (callerSequence == nullptr) {
		log.error() << "This sequence does not have a caller";
		return nullptr;
	}
	else return callerSequence;
}

std::vector<BaseSequence*> BaseSequence::getCallerStack() const {return callerStack;}

void BaseSequence::restartSequence() {
	runningState = SequenceState::restarting;
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