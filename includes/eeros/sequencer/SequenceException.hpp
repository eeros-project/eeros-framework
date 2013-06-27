#ifndef ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_

#include <eeros/sequencer/Sequence.hpp>

namespace eeros{
	namespace sequencer{
		class SequenceException{
			friend class Sequence;
		public:
			SequenceException(Sequence::method causingMethod, Sequence::method nextMethod);
		private:
			Sequence::method causingMethod;
			Sequence::method nextMethod;
			std::string reason;

		};

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_