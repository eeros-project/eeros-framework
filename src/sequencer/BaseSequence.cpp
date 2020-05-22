#include <eeros/sequencer/BaseSequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>

namespace eeros {
	namespace sequencer {

		BaseSequence::BaseSequence(BaseSequence* caller, bool blocking) : BaseSequence(caller->seq, caller, blocking) { }
		
		BaseSequence::BaseSequence(Sequencer& seq, BaseSequence* caller, bool blocking) : 
			seq(seq), caller(caller), blocking(blocking), state(SequenceState::idle), log('X'), 
			monitorTimeout("timeout", this, conditionTimeout, SequenceProp::abort), monitorAbort("abort", this, conditionAbort, SequenceProp::abort),
			pollingTime(100)
		{
			if (caller != nullptr) {
				callerStack = caller->getCallerStack();
				callerStack.push_back(caller);	// add latest caller
			}
			addMonitor(&monitorTimeout);	// default monitor
			addMonitor(&monitorAbort);	// default monitor
		}

		BaseSequence::~BaseSequence() { }

		int BaseSequence::action() {
			int retVal = -1;
			auto& seq = Sequencer::instance();
			if (seq.stepping) {
				log.warn() << "wait for next step command";
				while (Sequencer::running && !seq.nextStep);
				seq.nextStep = false;
			}
			state = SequenceState::running;
			checkActiveMonitor();	// check if this or a caller sequence is 'exceptionIsActive', set state accordingly

// 			if ((caller != NULL) && (state == SequenceState::running) && (caller->state == SequenceState::restarting)) 
// 				state = SequenceState::aborting;
			
			do {	//for restarting
				if (state == SequenceState::restarting) {	// sequence got restarted
					log.info() << "restart sequence '" << name << "'";
					state = SequenceState::running;
					resetTimeout();
				} 
				bool firstCheck = true;
				if (checkPreCondition()) {
// 					usleep(pollingTime*100);
					checkMonitorsOfBlockedCallers();	// check for monitors of all callers, start exception sequence, mark owner of fired monitor with 'exceptionIsActive'
					checkActiveMonitor();		// check if this or a caller sequence is 'exceptionIsActive', set state accordingly
					if (state == SequenceState::running) {
						log.info() << "start '" << name <<"'";
						retVal = action();	// call to custom implementation of method
					}
					while (state == SequenceState::running) {
						if (!firstCheck) {
							if (checkExitCondition()) state = SequenceState::terminated;	// check exit condition
							if (state != SequenceState::terminated) {
								checkMonitorsOfThisSequence();		// sets activeException if needed
								checkMonitorsOfBlockedCallers();	// sets activeException if needed
// 								usleep(pollingTime*1000);
							}
						} else firstCheck = false;
						
						checkActiveMonitor();					// sets state according to activeException
						if (state == SequenceState::running) usleep(pollingTime*1000);
					}
				} else { state = SequenceState::terminated;}
			} while (state == SequenceState::restarting);
			
			if (state == SequenceState::aborting) state = SequenceState::aborted;
			else state = SequenceState::terminated;
			if (Sequencer::instance().stepping) log.warn() << "'" << name << "' done";
			return retVal;
		}

		void BaseSequence::addMonitor(Monitor* monitor) {monitors.push_back(monitor);}

		std::vector<Monitor*> BaseSequence::getMonitors() const {return monitors;}

		void BaseSequence::checkMonitorsOfThisSequence() {
			log.trace() << "check monitor of seq " << name;
			for (Monitor* m : getMonitors()) checkMonitor(m);
		}

		void BaseSequence::checkMonitorsOfBlockedCallers() {
// 			log.trace() << "check monitor of blocked callers in seq "  << name << " state is " << getRunningState();
			if (!callerStackCreated) {	// when first called, caller stack has to be created
				if (this->blocking) {	// nonblocking sequences have no blocking callers
					std::vector<BaseSequence*> tempStack;
					log.trace() << "caller stack of seq '" << name <<"'";
					for (int i = callerStack.size() - 1; i >= 0; i--) {	// reverse iteration
						BaseSequence* entry = callerStack[i];
						log.trace() << "  entry '" << entry->getName() << "'";
						tempStack.push_back(entry);
						if (!entry->blocking) break;	// stop, if caller is nonblocking
					}
// 					log.trace() << "reverse caller stack of seq '" << name << "'";
					for (int i = tempStack.size(); i--;) {		// reverse vector
						callerStackReverse.push_back(tempStack[i]);
// 						log.trace() << "  entry '" << tempStack[i]->getName() << "'";
					}
				}
				callerStackCreated = true;
			}
			for (BaseSequence* s : callerStackReverse) {
				if (!s->inExcProcessing) 
					for (Monitor* m : s->getMonitors()) checkMonitor(m);
			}
		}

		void BaseSequence::checkMonitor(Monitor* m) {
			if (m->checkCondition()) {
				BaseSequence* owner = m->getOwner();
				if (!owner->monitorFired) {	// fire only once, is cleared after exception processing
					log.info() << "monitor '" << m->name << " of " << owner->getName() << "' fired";
					switch (m->getBehavior()) {
						case SequenceProp::resume : break;
						case SequenceProp::abort : 
						case SequenceProp::restart : {
// 							log.trace() << "set state and active monitor in seq " << owner->name << " state is " << owner->getRunningState();
							switch (m->getBehavior()) {
								case SequenceProp::abort : owner->state = SequenceState::aborting; break;
								case SequenceProp::restart : owner->state = SequenceState::restarting; break;
								default : break;
							}
							owner->monitorFired = true;
							owner->activeMonitor = m;
							break;
						}
						default : break;
					}
					owner->inExcProcessing = true;
					m->startExceptionSequence();	// start only if not yet in exception processing, blocking
					owner->inExcProcessing = false;
				}
			}
		}

		void BaseSequence::clearActiveMonitor() {
			monitorFired = false;
			activeMonitor = nullptr;
		}

		void BaseSequence::checkActiveMonitor() {
// 			log.trace() << "check active monitor in seq " << name << " state is " << getRunningState();
			if (monitorFired) {	// this sequence got the order to abort, restart ...
				switch (activeMonitor->getBehavior()) {
					case SequenceProp::resume : break;
					case SequenceProp::abort : state = SequenceState::aborting; break;
					case SequenceProp::restart : state = SequenceState::restarting; break;
					default : break;
				}
				clearActiveMonitor();
			} else {				// a caller got the order to abort, restart ...
				for (BaseSequence* s : getCallerStack()) {
					if (s->monitorFired && !s->inExcProcessing) {
						switch (s->activeMonitor->getBehavior()) {
							case SequenceProp::resume : break;
							case SequenceProp::abort : 
							case SequenceProp::restart : {	
								// this sequence as well as all the callers up to the calling sequences whose monitor fired have to be aborted
								state = SequenceState::aborting;
								BaseSequence* s = caller;
								while (s != nullptr && !s->monitorFired) {
									s->state = SequenceState::aborting;
									s = s->caller;
								}
								break;
							}
							default : break;
						}
// 						log.trace() << "state of seq " << name << " is "  << state;
					}
				}
			}
		}

		bool BaseSequence::checkExitCondition() {return true;}

		bool BaseSequence::checkPreCondition() {return true;}

		void BaseSequence::setName(std::string name) {this->name = name;}

		std::string BaseSequence::getName() const {return name;}

		SequenceState BaseSequence::getRunningState() const {return state;}

		void BaseSequence::setId(int id) {this->id = id;}

		int BaseSequence::getId() const {return id;}

		BaseSequence* BaseSequence::getCallerSequence() {
			if (caller == nullptr) {
				log.error() << "'" << name << "' does not have a caller";
				return nullptr;
			}
			else return caller;
		}

		std::vector<BaseSequence*> BaseSequence::getCallerStack() const {return callerStack;}

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

		void BaseSequence::setTimeoutExceptionSequence(BaseSequence& sequence) {
			monitorTimeout.setExceptionSequence(sequence);
		}

		void BaseSequence::resetAbort() {
			conditionAbort.reset();
		}

		std::ostream& operator<<(std::ostream& os, SequenceState state) {
			switch (state) {
				case SequenceState::idle: os << "idle"; break;
				case SequenceState::running: os << "running"; break;
				case SequenceState::paused: os << "paused"; break;
				case SequenceState::aborting: os << "aborting"; break;
				case SequenceState::aborted: os << "aborted"; break;
				case SequenceState::terminated: os << "terminated"; break;
				case SequenceState::restarting: os << "restarting"; break;
				default : break;
			}
			return os;
		}
	};
};
