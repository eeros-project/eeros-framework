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
	
	seq.addSequence(this);	// register in sequencer
	log.trace() << "Sequence '" << name << "' created";
}

Sequence::~Sequence() { }

void Sequence::run() {	// runs in thread
	std::ostringstream s;
	s << thread->get_id();
	std::string id = s.str();
	log.trace() << "Thread '" << id << "' for sequence '" << name << "' started";
	std::unique_lock<std::mutex> lk(m);
	cv.wait(lk);	// sends sequence to sleep, wait for start()
	lk.unlock();
	if (isMainSequence) log.info() << "sequence '" << name << "' started non-blocking";
	else log.info() << "sequence '" << name << "' started non-blocking, caller sequence: '" << callerSequence->getName() << "'";
	BaseSequence::action();
	log.info() << "sequence '" << name << "' terminated";
	log.trace() << "Thread '" << id << "' finished.";
}

int Sequence::start() {
	resetTimeout();
	if (isBlocking()) {	// starts action() blocking
		if (isMainSequence) log.info() << "sequence '" << name << "' started blocking";
		else log.info() << "sequence '" << name << "' started blocking, caller sequence: '" << callerSequence->getName() << "'";
		BaseSequence::action();				//action gets overwritten by child class
		log.info() << "sequence '" << name << "' terminated";
	} else {
		thread = new std::thread([this]() {this->run();});
		sleep(1);	// TODO muss weg
		cv.notify_one();	// starts actionFramework() in thread
	}
	return 0;
}

bool Sequence::isStep() {
	return false;
}

