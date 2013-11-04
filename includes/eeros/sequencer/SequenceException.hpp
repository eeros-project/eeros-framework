#ifndef ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_

#include <eeros/sequencer/Sequence.hpp>

#include <string>

namespace eeros{
	namespace sequencer{
		class ErrorHandler;

		/**
		 * This Exception can be thrown. 
		 * You can define how the system have to continue the excecution in the sequence by creating a ErrorHandler.
		 */
		class SequenceException{
			friend class Sequence;
		public:
			/**
			 * Create an exception containing only a message string reason
			 */
			SequenceException(std::string reason);
			
			/**
			 * Create an exception containing 
			 * the sequence seqCause which caused the exception 
			 * the sequence methode cause, which caused the exception 
			 * the sequence methode next, where the execution should continue
			 * the ErrorHandler error, which does some work before starting the next method.
			 * By setting toBegin to true, the sequence will be restarted and the flag goToNext will be ignored
			 * If toBegin is false, then you can define goToNext with true, so the sequence starts at the method next, 
			 * it is very important, that this method is in the callback list.
			 * message string reason
			 */
			SequenceException(Sequence* seqCause, Sequence::method cause, Sequence::method next,
				              ErrorHandler* error, bool toBegin, bool goToNext, std::string reason);
			
			/**
			 * The destructor deletes the ErrorHandler
			 */
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