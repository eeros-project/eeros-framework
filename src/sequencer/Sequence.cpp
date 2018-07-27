#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>
#include <sys/syscall.h>

using namespace eeros;
using namespace eeros::sequencer;

Sequence::Sequence(std::string name, Sequencer& seq) : Sequence(name, seq, nullptr, false) { }

Sequence::Sequence(std::string name, Sequencer& seq, BaseSequence* caller, bool blocking) : BaseSequence(seq, caller, blocking) {
	static int sequenceCount;
	setId(sequenceCount++);	//TODO check how many sequence objects of this type are allowed. Maybe singleton.
	if (name == "") {
		throw Fault("all sequences must have a name");
	} else {
		for (Sequence *s : seq.getListOfAllSequences()) {
			if ( this->name == s->getName() ) throw Fault("all sequences must have different names"); 
		}
		this->name = name;
	}
	
	seq.addSequence(*this);	// register in sequencer
	if (!blocking) t = new std::thread([this]() {this->run();});
	log.trace() << "sequence '" << name << "' created";
}

Sequence::~Sequence() {/*running = false;*/}

void Sequence::run() {	// runs in thread
	struct sched_param schedulingParam;
	schedulingParam.sched_priority = 0;
	if (sched_setscheduler(0, SCHED_OTHER, &schedulingParam) != 0) log.error() << "could not set scheduling parameter for sequence thread";
	sched_getparam(0, &schedulingParam);
	std::ostringstream s;
	s << t->get_id();
	std::string id = s.str();
	log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " for sequence '" << name << "' and with priority " << schedulingParam.sched_priority << " started";
	while (running) {
		while(running && !go) usleep(1000);
		if (!running) break;
		go = false;
		log.info() << "sequence '" << name << "' (non-blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
		BaseSequence::action();
		log.info() << "sequence '" << name << "' terminated";
		done = true;
	}
	log.trace() << "thread " << getpid() << ":" << syscall(SYS_gettid) << " finished.";
}

int Sequence::start() {
	resetTimeout();
	resetAbort();
	Sequencer::running = true;
	if (blocking) {	// starts action() blocking
		log.info() << "sequence '" << name << "' (blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
		BaseSequence::action();				//action gets overwritten by child class
		log.info() << "sequence '" << name << "' terminated";
	} else {
		go = true;
		done = false;
	}
	return 0;
}

void Sequence::wait() {
	if (t != nullptr) {
		while (!done) usleep(1000);	// wait for thread to finish current run
	}
}

void Sequence::waitAndTerminate() {
	if (t != nullptr) {
		while (!done) usleep(1000);	// wait for thread to finish current run
		running = false;
		if (t->joinable()) t->join();
	}
}

