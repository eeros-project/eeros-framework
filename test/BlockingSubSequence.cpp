#include "BlockingSubSequence.hpp"

#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>


BlockingSubSequence::BlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller) 
	: eeros::sequencer::Sequence(name, caller){
}


BlockingSubSequence::~BlockingSubSequence(void){
}

void BlockingSubSequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&BlockingSubSequence::MoveToA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&BlockingSubSequence::MoveToB));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&BlockingSubSequence::MoveToC));
}

void BlockingSubSequence::MoveToA(){
	calledMethode.append("MoveToA ");
}

void BlockingSubSequence::MoveToB(){
	calledMethode.append("MoveToB ");
}

void BlockingSubSequence::MoveToC(){
	calledMethode.append("MoveToC ");
}

std::string BlockingSubSequence::getCalledMethode(){
	return calledMethode;
}