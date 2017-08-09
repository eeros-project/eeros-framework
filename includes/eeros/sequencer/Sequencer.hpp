#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <eeros/logger/Logger.hpp>
#include <vector>
#include <atomic>

namespace eeros {
	namespace sequencer {
		
// 		namespace state {
// 			enum type { executing, waiting, terminating, terminated, idle };
// 		}
// 		
// 		namespace mode {
// 			enum type { automatic, stepping };
// 		}
	
		class Sequence;

		class Sequencer {
			Sequencer();
		public:
			virtual ~Sequencer();
			static Sequencer& instance();
			
			void addSequence(Sequence& seq);
			void addMainSequence(Sequence& mainSeq);
			Sequence* getMainSequence();
			Sequence* getSequenceById(int id);
			Sequence* getSequenceByName(std::string name);
			std::vector< Sequence* > getListOfAllSequences();
		private:
			Sequence* mainSequence;
			std::vector<Sequence*> sequenceList;	//list of all sequences
			eeros::logger::Logger log;	
			unsigned int id;
		};
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
