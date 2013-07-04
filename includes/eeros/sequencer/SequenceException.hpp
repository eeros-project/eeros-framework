#ifndef ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_

#include <eeros/sequencer/Sequence.hpp>

#include <string>

namespace eeros{
	namespace sequencer{
		class ErrorHandler;

		class SequenceException{
			friend class Sequence;
		public:
			SequenceException(std::string reason);
			SequenceException(Sequence* seqCause, Sequence::method cause, Sequence::method next,
				              ErrorHandler* error, bool toBegin, bool goToNext, std::string reason);
			~SequenceException();
		private:
			Sequence* seqenceCausing;
			Sequence::method causingMethod;
			Sequence::method nextMethod;
			ErrorHandler* errorHandler;
			std::string message;
			bool returnToBegin;
			bool goToNext;

		};

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_