#ifndef ORG_EEROS_SEQUENCER_STEP_HPP_
#define ORG_EEROS_SEQUENCER_STEP_HPP_

#include <eeros/sequencer/BaseSequence.hpp>

namespace eeros {
	namespace sequencer {

		class Step : public BaseSequence {
		public:
			Step(std::string name, Sequencer& seq, BaseSequence* caller) : BaseSequence(seq, caller) {this->name = name;}
			virtual ~Step() { };
			
			virtual int operator() () = 0;	// this operator has to be implemented in the derived sequence
			virtual int action() = 0;	// this function has to be implemented in the derived sequence
			int start() {
				resetTimeout();
				if (!exceptionIsActive) log.info() << "step '" << name << "' started";
				BaseSequence::action();
				return 0;
			}
			bool isStep() const {return true;};
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_STEP_HPP_
