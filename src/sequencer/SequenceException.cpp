#include <eeros/sequencer/SequenceException.hpp>
#include <eeros/sequencer/Sequence.hpp>

using namespace eeros::sequencer;

SequenceException::SequenceException(std::string message,
									 Sequence* exceptionHandler,
									 ExceptionReturnBehavior returnBehavior,
									 Sequence* ensuingSequence) : 
									 message(message),
									 exceptionHandler(exceptionHandler),
									 returnBehavior(returnBehavior),
									 ensuingSequence(ensuingSequence),
									 EEROSException(message) {
	// nothing to do
}

SequenceException::~SequenceException() throw() { }

const char* SequenceException::what() const throw() {
	return message.c_str();
}

void SequenceException::handle() const {
//	exceptionHandler->run(); TODO
}

ExceptionReturnBehavior SequenceException::getReturnBehavior() const throw() {
	return returnBehavior;
}

Sequence* SequenceException::getEnsuingSequence() const throw() {
	return ensuingSequence;
}
