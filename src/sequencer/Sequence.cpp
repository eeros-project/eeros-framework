#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;

Sequence::Sequence(std::string name, Sequencer& seq) : Sequence(name, seq, nullptr) { }

Sequence::Sequence(std::string name, Sequencer& seq, BaseSequence* caller) : BaseSequence(seq, caller) {
	static int sequenceCount;
	setId(sequenceCount++);	//TODO check how many sequence objects of this type are allowed. Maybe singleton.
	if (name == "") {
		throw Fault("All sequences must have a name");
	} else {
		for (Sequence *s : seq.getListOfAllSequences()) {
			if ( this->name == s->getName() ) throw Fault("All sequences must have different names"); 
		}
		this->name = name;
	}
	
	seq.addSequence(*this);	// register in sequencer
	log.trace() << "Sequence '" << name << "' created";
}

Sequence::~Sequence() { }

void Sequence::run() {	// runs in thread
	struct sched_param schedulingParam;
	schedulingParam.sched_priority = 0;
	if (sched_setscheduler(0, SCHED_OTHER, &schedulingParam) != 0) log.error() << "could not set scheduling parameter for sequence thread";
	sched_getparam(0, &schedulingParam);
	std::ostringstream s;
	s << thread->get_id();
	std::string id = s.str();
	log.trace() << "Thread '" << id << "' for sequence '" << name << "' and with prio=" << schedulingParam.sched_priority << " started";
	log.info() << "sequence '" << name << "' (non-blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
	BaseSequence::action();
	log.info() << "sequence '" << name << "' terminated";
	log.trace() << "Thread '" << id << "' finished.";
}

int Sequence::start() {
	resetTimeout();
	if (isBlocking()) {	// starts action() blocking
		log.info() << "sequence '" << name << "' (blocking), caller sequence: '" << ((caller != nullptr)?caller->getName():"no caller") << "'";
		BaseSequence::action();				//action gets overwritten by child class
		log.info() << "sequence '" << name << "' terminated";
	} else {
		thread.reset(new std::thread([this]() {this->run();}));
	}
	return 0;
}

void Sequence::join() {
	if (thread != nullptr) {
		if (thread->joinable())
			thread->join();
	}
}

