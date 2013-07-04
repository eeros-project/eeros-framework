#include <eeros/sequencer/SequenceException.hpp>

#include <eeros/sequencer/ErrorHandler.hpp>

using namespace eeros::sequencer;


SequenceException::SequenceException(Sequence* seqCause, Sequence::method cause, Sequence::method next,
	                                 ErrorHandler* error, bool toBegin, bool goNext, std::string reason)
	: seqenceCausing(seqCause),
	  causingMethod(cause),
	  nextMethod(next),
	  errorHandler(error),
	  returnToBegin(toBegin),
	  goToNext(goNext),
	  message(reason){
}

SequenceException::SequenceException(std::string reason) 
	: seqenceCausing(0),
	  causingMethod(0),
	  nextMethod(0),
	  errorHandler(0),
	  returnToBegin(false),
	  goToNext(false),
	  message(reason){
	
}

SequenceException::~SequenceException(){
	if (errorHandler){
		delete errorHandler;
	}
}