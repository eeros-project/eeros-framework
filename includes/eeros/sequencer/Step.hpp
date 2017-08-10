#ifndef ORG_EEROS_SEQUENCER_STEP_HPP_
#define ORG_EEROS_SEQUENCER_STEP_HPP_

#include <eeros/sequencer/BaseSequence.hpp>

namespace eeros {
	namespace sequencer {

		class Step : public BaseSequence {
		public:
			Step(std::string name, Sequencer& seq, BaseSequence* caller) : BaseSequence(seq, caller) {this->name = name;}
			virtual ~Step() { };
			
			virtual int operator() () {return start();}	// this operator can be overloaded in the derived sequence
			virtual int action() = 0;		// this function has to be implemented in the derived sequence
			int start() {
				resetTimeout();
				if (!exceptionIsActive) log.info() << "step '" << name << "' started";
				BaseSequence::action();
				return 0;
			}
			void setBlocking() {blocking = true;}
			void setNonBlocking() {log.error() << "a step is always blocking";}
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_STEP_HPP_
