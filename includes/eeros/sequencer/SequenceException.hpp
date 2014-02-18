#ifndef ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_

#include <string>
#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace sequencer {
		
		// forward declarations
		class Sequence;
		
		// enums
		enum ExceptionReturnBehavior { kContinue, kRepeatStep, kRepeatSequence, kNewSequence };
		
		class SequenceException : public eeros::EEROSException {

		public:
			SequenceException(std::string message, Sequence* exceptionHandler, ExceptionReturnBehavior returnBehavior = kRepeatStep, Sequence* ensuingSequence = nullptr);
			virtual ~SequenceException() throw();
			
			virtual ExceptionReturnBehavior getReturnBehavior() const throw();
			virtual Sequence* getEnsuingSequence() const throw();
			
			virtual const char* what() const throw();
			virtual void handle() const;
			
		private:
			std::string message;
			Sequence* exceptionHandler;
			ExceptionReturnBehavior returnBehavior;
			Sequence* ensuingSequence;
		};
	};
};

#endif // ORG_EEROS_SEQUENCER_SEQUENCEEXCEPTION_HPP_
