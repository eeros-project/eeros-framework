//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequencer.hpp>

#include "MyNonBlockingSubSequence.hpp"

MyNonBlockingSubSequence::MyNonBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller) 
	: eeros::sequencer::Sequence(name, caller){
		callerThread.addRunnable(this);
}


MyNonBlockingSubSequence::~MyNonBlockingSubSequence(void){
}

void MyNonBlockingSubSequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToAA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToBB));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToCC));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::Stopping));
}

void MyNonBlockingSubSequence::MoveToAA(){
	std::cout << "MoveToA" << std::endl;
	std::cout << "Going to MoveToB" << std::endl;
}

void MyNonBlockingSubSequence::MoveToBB(){
	std::cout << "MoveToB" << std::endl;
	std::cout << "Going to MoveToC" << std::endl;
}

void MyNonBlockingSubSequence::MoveToCC(){
	std::cout << "MoveToC" << std::endl;
	std::cout << "Going to Stopping" << std::endl;
}

void MyNonBlockingSubSequence::Stopping(){
	std::cout << "Stopping" << std::endl;
	std::cout << "End of Sequence!" << this->getName() << std::endl;
	callerThread.stop();
}
