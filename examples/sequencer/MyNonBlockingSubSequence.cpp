#include "MyNonBlockingSubSequence.hpp"

//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

#include "MyErrorHandlerA.hpp"

using namespace eeros::examples::sequencer;

MyNonBlockingSubSequence::MyNonBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller) 
	: eeros::sequencer::Sequence(name, caller){
	callerThread.addRunnable(this);
}


MyNonBlockingSubSequence::~MyNonBlockingSubSequence(void){
}

void MyNonBlockingSubSequence::fillCallBacks(){
	fillVersion1();
}

void MyNonBlockingSubSequence::fillVersion1(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToAA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToBB));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToCC));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::Stopping));
}

void MyNonBlockingSubSequence::fillVersion2(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToAA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToException));
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

void MyNonBlockingSubSequence::MoveToException(){
//For Test
	static int i = 0;
	std::cout << "MoveException" << std::endl;
	std::cout << "Going to Next" << std::endl;
	if(i == 0){
		i++;
		std::cout << "Going to ErrorHandler" << std::endl;
		MyErrorHandlerA* errorHandlerA = dynamic_cast<MyErrorHandlerA*>(eeros::sequencer::ErrorHandler::getErrorHandler("MyErrorHandlerA"));
		if(!errorHandlerA){
			errorHandlerA = new MyErrorHandlerA("MyErrorHandlerA");
		}
		//case 1 and 3
		throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToException), 0,
			                                          errorHandlerA, true, false, "TestException");
													  
		//case 2 and 4
		/*throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MyNonBlockingSubSequence::MoveToException), 0, 
			                                          errorHandlerA, false, false, "TestException");
													  */
	}
}