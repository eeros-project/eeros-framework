#include <eeros/sequencer/Sequence.hpp>

using namespace eeros::sequencer;

SequenceBase::SequenceBase(std::string name, Sequencer* sequencer) : name(name), sequencer(sequencer) { }

std::string SequenceBase::getName() const {
	return name;
}

bool SequenceBase::checkPreCondition() {
	return true;
}
bool SequenceBase::checkPostCondition() {
	return true;
}

void SequenceBase::yield() {
	if(sequencer != nullptr) {
		sequencer->yield();
	}
}

void SequenceBase::init() { }

void SequenceBase::exit() { }
