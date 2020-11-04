#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>
#include <sys/syscall.h>
#include <future>

namespace eeros {
namespace sequencer {

Sequence::Sequence(std::string name, Sequencer& seq) : Sequence(name, seq, nullptr, false) { }

Sequence::Sequence(std::string name, BaseSequence* caller, bool blocking) : Sequence(name, caller->seq, caller, blocking) { }

Sequence::Sequence(std::string name, Sequencer& seq, BaseSequence* caller, bool blocking) : BaseSequence(seq, caller, blocking) {
  if (name == "") {
    throw Fault("all sequences must have a name");
  } else {
    for (Sequence *s : seq.getListOfAllSequences()) {
      if (name == s->getName()) throw Fault("all sequences must have different names"); 
    }
    this->name = name;
  }
  seq.addSequence(*this);	// register in sequencer
  log.trace() << "sequence '" << name << "' created";
}

int Sequence::run() {	// runs in thread
  struct sched_param schedulingParam;
  schedulingParam.sched_priority = 0;
  if (sched_setscheduler(0, SCHED_OTHER, &schedulingParam) != 0) log.error() << "could not set scheduling parameter for sequence thread";
  sched_getparam(0, &schedulingParam);
  std::ostringstream s;
  log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " for sequence '" << name << "' and with priority " << schedulingParam.sched_priority << " started";
  int retVal;
  log.info() << "start thread for sequence '" << name << "' (non-blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
  retVal = BaseSequence::action();
  log.info() << "sequence '" << name << "' terminated";
  log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " finished.";
  return retVal;
}

int Sequence::start() {
  int retVal = -1;
  resetTimeout();
  resetAbort();
  Sequencer::running = true;
  if (blocking) {	// starts action() blocking
    log.info() << "create sequence '" << name << "' (blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
    retVal = BaseSequence::action();
    log.info() << "sequence '" << name << "' terminated";
  } else {
    fut = std::async(std::launch::async, &Sequence::run, this);
  }
  return retVal;
}

int Sequence::getResult() {
  return retVal;
}

void Sequence::wait() {
  if (fut.valid()) retVal = fut.get();
}

} // namespace sequencer
} // namespace eeros
