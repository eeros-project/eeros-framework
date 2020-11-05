#include <eeros/sequencer/BaseSequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>

namespace eeros {
namespace sequencer {

BaseSequence::BaseSequence(BaseSequence* caller, bool blocking) : BaseSequence(caller->seq, caller, blocking) { }

BaseSequence::BaseSequence(Sequencer& seq, BaseSequence* caller, bool blocking) 
    : seq(seq), caller(caller), blocking(blocking), state(SequenceState::idle), log('X'), 
      monitorTimeout("timeout", this, conditionTimeout, SequenceProp::abort), monitorAbort("abort", this, conditionAbort, SequenceProp::abort),
      pollingTime(100) {
  if (caller != nullptr) {
    callerStack = caller->callerStack;
  }
  callerStack.insert(callerStack.begin(), this); // add this sequence at the front
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
  std::lock_guard<std::mutex> lock(mtx);
  while (state != SequenceState::terminated) {
    switch (state) {
      case SequenceState::idle: { // upon creation
        clearActiveMonitor();
        checkMonitors(); // check if this or a caller sequence is 'exceptionIsActive', set state accordingly
        if (!(state == SequenceState::aborting || state == SequenceState::restarting)) state = SequenceState::starting;
        break;
      }
      case SequenceState::starting: { // started, before first run
        if (checkPreCondition()) {
          log.info() << "start '" << name <<"'";
          state = SequenceState::running; // change of state, though call to action will block!!!
          retVal = action();  // call to custom implementation of method
        } else {
          state = SequenceState::terminated;
        }
        break;
      }
      case SequenceState::running: { // active and running, eigentlich checking
        checkMonitors();    // check monitors of this sequence and all callers, execute exception if necessary
        if (state == SequenceState::restarting) continue; // stop any further actions when restarting
        if (checkExitCondition()) state = SequenceState::terminated;
        if (state == SequenceState::running) usleep(pollingTime * 1000);  // wait only in case of normal execution
        break;
      }
      case SequenceState::paused: { // not used
        break;
      }
      case SequenceState::aborting: { // to be stopped, due to caller sequence
        state = SequenceState::terminated;
        break;
      }
      case SequenceState::restarting: { // sequence got restarted
        log.info() << "restart sequence '" << name << "'";
        state = SequenceState::starting;
        clearActiveMonitor();
        resetTimeout();
        break;
      }
      default:
        log.error() << "undefined sequence state";
    }
  }
  state = SequenceState::idle;
  return retVal;
}

/* 
 * Check all the monitors of this sequence and of the caller sequences of this sequence.
 * Register, if one of the monitors fired. If so, put all the sequences in the caller stack
 * starting from the this sequence up to the sequence which holds the monitor which fired into
 * state 'aborting'. Reason: the monitor tells its own sequence and all its callees that they 
 * either have to be aborted (in case of abort or restart) or continue to run (in case of resume).
 */
void BaseSequence::checkMonitors() {
  BaseSequence* activeSeq = nullptr;
  for (BaseSequence* s : callerStack) {
    if (!s->inExcProcessing) {    // while in exception processing, the same monitor must not fire again
      for (Monitor* m : s->getMonitors()) {
        auto val = checkMonitor(m);
        if (val != nullptr) activeSeq = val;
      }
    }
  }

  if (activeSeq != nullptr) {
    log.trace() << "fired detected, prop = " << activeSeq->activeMonitor->behavior << ", handle all callers of " << name;
    for (BaseSequence* s : callerStack) {
      log.trace() << "\thandle " << s->name << ", monitorFired = " << s->monitorFired << ", prop = " << activeSeq->activeMonitor->getBehavior();
      if (!s->monitorFired) {
        SequenceProp prop = activeSeq->activeMonitor->getBehavior();
        if (prop == SequenceProp::abort || prop == SequenceProp::restart) {
          s->state = SequenceState::aborting;
          log.trace() << "\tput " << s->name << " into state " << s->state;
        }
      } else break;
    }
  }
}

/*
 * Check if monitor has fired.  
 * Set state and active monitor of owner sequence and start exception sequence
 * If a monitor fires, the exception sequence must run directly and halt the 
 * current sequence and its callees.
 * Returns a pointer to the sequence if a monitor fired, else nullptr
 */
BaseSequence* BaseSequence::checkMonitor(Monitor* m) {
  BaseSequence* firedSeq = nullptr;
  log.trace() << "check monitor "  << m->name << " of " << m->getOwner()->name;
  if (m->checkCondition()) {
    BaseSequence* owner = m->getOwner();
    firedSeq = owner;
    if (!owner->monitorFired) { // fire only once, is cleared after exception processing
      log.info() << "monitor '" << m->name << "' of '" << owner->getName() << "' fired";
      owner->monitorFired = true;
      owner->activeMonitor = m;
      owner->inExcProcessing = true;
      m->startExceptionSequence();  // start only if not yet in exception processing, blocking
      owner->inExcProcessing = false;
      switch (m->getBehavior()) {
        case SequenceProp::resume: owner->state = SequenceState::running; owner->monitorFired = false; break;
        case SequenceProp::abort: owner->state = SequenceState::aborting; break;
        case SequenceProp::restart: owner->state = SequenceState::restarting; break;
        default : break;
      }
      log.trace() << "after exception, put " << owner->getName() << " into state " << owner->state;
    }
  }
  return firedSeq;
}

void BaseSequence::addMonitor(Monitor* monitor) {monitors.push_back(monitor);}

std::vector<Monitor*> BaseSequence::getMonitors() const {return monitors;}

void BaseSequence::clearActiveMonitor() {
  log.trace() << "clear active monitor in " << name;
  monitorFired = false;
  activeMonitor = nullptr;
}

bool BaseSequence::checkExitCondition() {return true;}

bool BaseSequence::checkPreCondition() {return true;}

void BaseSequence::setName(std::string name) {this->name = name;}

std::string BaseSequence::getName() const {return name;}

void BaseSequence::setId(int id) {this->id = id;}

int BaseSequence::getId() const {return id;}

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

void BaseSequence::abort() {
  conditionAbort.set();
}

std::ostream& operator<<(std::ostream& os, const SequenceState& state) {
  switch (state) {
    case SequenceState::idle: os << "idle"; break;
    case SequenceState::starting: os << "starting"; break;
    case SequenceState::running: os << "running"; break;
    case SequenceState::paused: os << "paused"; break;
    case SequenceState::aborting: os << "aborting"; break;
    case SequenceState::terminated: os << "terminated"; break;
    case SequenceState::restarting: os << "restarting"; break;
    default : break;
  }
  return os;
}

}
}
