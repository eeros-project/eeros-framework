#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <eeros/logger/Logger.hpp>
#include <vector>
#include <atomic>
#include <memory>

namespace eeros {
	namespace sequencer {
		
		class Sequence;
		class SequencerUI;

		class Sequencer {
			friend class BaseSequence;
			friend class SequencerUI;
			
			Sequencer();
		public:
			virtual ~Sequencer();
			static Sequencer& instance();
			
			void addSequence(Sequence& seq);
			Sequence* getSequenceById(int id);
			Sequence* getSequenceByName(std::string name);
			std::vector<Sequence*> getListOfAllSequences();
			/**
			* Waits for the sequencer to terminate all its sequences together with their associated threads
			*/
			void wait();
			void abort();
			void singleStepping();
			static bool running;
		private:
			void step();
			void restart();
			Sequence* mainSequence;
			std::vector<Sequence*> sequenceList;	// list of all sequences
			eeros::logger::Logger log;	
			unsigned int id;
			bool stepping;
			bool nextStep;
			SequencerUI* ui;
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
