#include <eeros/sequencer/Monitor.hpp>
#include <eeros/sequencer/BaseSequence.hpp>

namespace eeros {
	namespace sequencer {

		Monitor::Monitor(std::string name, BaseSequence* owner, Condition& condition, SequenceProp behavior, BaseSequence* exceptionSequence)
		: owner(owner), exceptionSequence(exceptionSequence), condition(condition), behavior(behavior), name(name)
		{ }

		Monitor::~Monitor() { }

		void Monitor::startExceptionSequence() {
			if (exceptionSequence != nullptr) {
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

		std::ostream& operator<<(std::ostream& os, SequenceProp prop) {
			switch (prop) {
				case SequenceProp::resume: os << "resume"; break;
				case SequenceProp::abort: os << "abort"; break;
				case SequenceProp::restart: os << "restart"; break;
				default : break;
			}
			return os;
		}
	};
};
