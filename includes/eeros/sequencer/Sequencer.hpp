#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <eeros/core/Executor.hpp>

class TimeDomain;

namespace eeros{
	namespace sequencer{

		class Sequencer : public Executor
		{
		public:
			static Sequencer* getMainSequencer();
			Sequencer(std::string name);
			virtual ~Sequencer();
			void addTimeDomain(TimeDomain* tDomain);
			void deleteAllTimeDomains();
			void addSubSequencer(Sequencer* seq);
			void deleteAllSubSequencers();
			std::string getName();
			Sequencer* findSequencer(std::string name);
			void deleteSequencer(std::string name);
		protected:
			
		private:
			static Sequencer* mainSequencer;
			//List for the mainSequencer, the only Object
			std::list<Sequencer*> subSequencers;
			std::list<TimeDomain*> timeDomains;
			std::string sequenceName;
		};//class Sequence

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCE_HPP_