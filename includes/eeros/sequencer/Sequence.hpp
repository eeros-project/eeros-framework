#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <eeros/core/Executor.hpp>

class TimeDomain;

namespace eeros{
	namespace sequencer{

		class Sequence : public Executor
		{
		public:
			typedef void (Sequence::*method)();

			Sequence(std::string name, TimeDomain* ptimeDomain);
			void addSubSequence(Sequence* seq);
			void deleteAllSubSequences();
			virtual void run();
			std::string getName();

		protected:
			TimeDomain* timeDomain;
			void next(method step);
			void run_state();
			Sequence* findSequence(std::string name);
		private:
			std::list<Sequence*> subSequences;
			std::string sequenceName;
			method fun;
		};//class Sequence

	};//namespace sequencer
};//namespace eeros
#endif