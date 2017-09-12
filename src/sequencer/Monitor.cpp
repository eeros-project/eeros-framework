#include <eeros/sequencer/Monitor.hpp>
#include <eeros/sequencer/BaseSequence.hpp>

using namespace eeros;
using namespace eeros::sequencer;

Monitor::Monitor(std::string name, BaseSequence* owner, Condition& condition, SequenceProp behavior, BaseSequence* exceptionSequence)
: name(name), owner(owner), condition(condition), behavior(behavior), exceptionSequence(exceptionSequence)
{ }

Monitor::~Monitor() { }

void Monitor::startExceptionSequence() {
	if (exceptionSequence != nullptr) {
		exceptionSequence->caller->inExcProcessing = true;
		exceptionSequence->start();
	}
}

void Monitor::setBehavior(SequenceProp behavior) {
	this->behavior = behavior;
}

SequenceProp Monitor::getBehavior() const {
	return behavior;
}

BaseSequence* Monitor::getOwner() const {
	return owner;
}

void Monitor::setExceptionSequence(BaseSequence& exceptionSequence) {
	this->exceptionSequence = &exceptionSequence;
}

bool Monitor::checkCondition() {
	return condition.isTrue();
}
