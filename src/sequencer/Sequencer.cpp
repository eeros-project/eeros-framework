#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

namespace eeros {
	namespace sequencer {
		bool Sequencer::running = true;

		Sequencer::Sequencer() : log('Z') { }

		Sequencer::~Sequencer() { }

		Sequencer& Sequencer::instance() {
			static Sequencer seq;
			return seq;
		}

		void Sequencer::addSequence(Sequence& seq) {
			sequenceList.push_back(&seq);
		}

		Sequence* Sequencer::getSequenceById(int id) {
			for (Sequence *seq : getListOfAllSequences()) {
				if (id == seq->getId()) return seq;
			}
			log.error() << "No sequence with id '" << id << "' found.";
			return nullptr;
		}

		Sequence* Sequencer::getSequenceByName(std::string name) {
			for (Sequence *seq : getListOfAllSequences()) {
				if ( name == seq->getName() ) return seq;
			}
			log.error() << "No sequence with name '" << name << "' found.";
			return nullptr;
		}

		std::vector<Sequence*> Sequencer::getListOfAllSequences() {
			return sequenceList;
		}
		
		void Sequencer::join() {
			std::vector<Sequence*> list = getListOfAllSequences();
			for (Sequence* s : list) s->join();
		}

		// can be used to terminate the threads of all sequences
		void Sequencer::abort() {
			std::vector<Sequence*> list = getListOfAllSequences();
			for (Sequence* s : list) s->conditionAbort.set();
			running = false;
		}
	};	//namespace sequencer
}; // namespace eeros
























