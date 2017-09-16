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

		void Sequencer::addMainSequence(Sequence& mainSeq) { 
			if (mainSeq.isBlocking()) throw Fault("Main sequence has to be a nonblocking sequence");
			mainSequence = &mainSeq;
		// 	mainSequence->start();
		}

		Sequence* Sequencer::getMainSequence() {
			if (mainSequence == nullptr) throw Fault("Main sequence not set in sequencer");
			return mainSequence;
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

		std::vector< Sequence* > Sequencer::getListOfAllSequences() {
			return sequenceList;
		}

		// can be used to terminate the thread of the main sequence, will destroy all threads of subsequences
		void Sequencer::abort() {
			getMainSequence()->conditionAbort.set();
			running = false;
		}
	};	//namespace sequencer
}; // namespace eeros
























