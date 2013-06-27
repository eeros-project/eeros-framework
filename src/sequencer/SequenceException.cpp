#include <eeros/sequencer/SequenceException.hpp>

eeros::sequencer::SequenceException::SequenceException(eeros::sequencer::Sequence::method cause, eeros::sequencer::Sequence::method next){
	causingMethod = cause;
	nextMethod = next;
}