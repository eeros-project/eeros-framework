#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <string>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequence.hpp>

namespace eeros {
	namespace sequencer {

		class Sequencer {
		public:
			Sequencer(std::string name, Sequence& mainSequence);
			virtual ~Sequencer();
			
			Sequence& getMainSequence();
			std::string getName();
			
			bool done();
			
		private:
            eeros::Executor sequenceExecutor;
			eeros::Executor timeoutExecutor;
			Sequence& mainSequence;
			std::string name;
			
			static constexpr double timeoutExecutorPeriod = 0.001;
		}; // class Sequence

	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_