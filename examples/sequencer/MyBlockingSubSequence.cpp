//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include "MyBlockingSubSequence.hpp"

MyBlockingSubSequence::MyBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller) 
	: eeros::sequencer::Sequence(name, caller){
}


MyBlockingSubSequence::~MyBlockingSubSequence(void){
}

void MyBlockingSubSequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToB));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToC));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::Stopping));
}

void MyBlockingSubSequence::MoveToA(){
	std::cout << "MoveToA" << std::endl;
	std::cout << "Going to MoveToB" << std::endl;
}

void MyBlockingSubSequence::MoveToB(){
	std::cout << "MoveToB" << std::endl;
	std::cout << "Going to MoveToC" << std::endl;
}

void MyBlockingSubSequence::MoveToC(){
	std::cout << "MoveToC" << std::endl;
	std::cout << "Going to Stopping" << std::endl;
}

void MyBlockingSubSequence::Stopping(){
	std::cout << "Stopping" << std::endl;
	std::cout << "End of Sequence!" << std::endl;
	//stop();
}
