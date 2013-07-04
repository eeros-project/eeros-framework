#include "MyBlockingSubSequence.hpp"

//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

#include "MyErrorHandlerA.hpp"

MyBlockingSubSequence::MyBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller) 
	: eeros::sequencer::Sequence(name, caller){
}


MyBlockingSubSequence::~MyBlockingSubSequence(void){
}

void MyBlockingSubSequence::fillCallBacks(){
	fillVersion2();
}

void MyBlockingSubSequence::fillVersion1(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToB));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToC));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::Stopping));
}

void MyBlockingSubSequence::fillVersion2(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToA));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToException));
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

void MyBlockingSubSequence::MoveToException(){
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
		throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToException), 0,
			                                          errorHandlerA, true, false, "TestException");
													  
		//case 2 and 4
		/*throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MyBlockingSubSequence::MoveToException), 0,
			                                          errorHandlerA, false, false, "TestException");
													  */
		
	}
}