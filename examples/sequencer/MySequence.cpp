#include "MySequence.hpp"

//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/Step.hpp>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

//#include "MySubSequencer.hpp"
#include "MySequencer.hpp"
#include "MyBlockingSubSequence.hpp"
#include "MyNonBlockingSubSequence.hpp"
#include "MyErrorHandlerA.hpp"

MySequence::MySequence(std::string name, eeros::sequencer::Sequencer& caller)
	: eeros::sequencer::Sequence(name, caller){
	callerThread.addRunnable(this);
}


MySequence::~MySequence(void){
}

void MySequence::fillCallBacks(){
	fillVersion5();
}

void MySequence::fillVersion1(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveNonBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Moving));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::WaitingForNonBlocking));
	//addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Moving));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

void MySequence::fillVersion2(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveException));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Moving));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

void MySequence::fillVersion3(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

void MySequence::fillVersion4(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveNonBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveException));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::WaitingForNonBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

void MySequence::fillVersion5(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveNonBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::WaitingForNonBlocking));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

void MySequence::Init(){
	std::cout << "Init" << std::endl;
	std::cout << "Going to Initialising" << std::endl;
}

/** Initialising
*/
void MySequence::Initialising(){
	std::cout << "Initialising" << std::endl;
	std::cout << "Going to Initialised" << std::endl;
	//Create blocks for control
	TimeDomain* timeDomain = new TimeDomain();

	//all TimeDomain are saved in the MainSequence not in callerThread
	eeros::sequencer::Sequencer::getMainSequencer()->addTimeDomain(timeDomain);

	Step* step = new Step(1.0, 5.0, 0.0);
	Gain* gain = new Gain(10);
	BlockOutput* output = new BlockOutput();
	gain->getIn().connect(step->getOut());
	output->getIn().connect(gain->getOut());

	//add the blocks to the time domain
	timeDomain->addBlock(step);
	timeDomain->addBlock(gain);
	timeDomain->addBlock(output);
}

/** Initialised
*/
void MySequence::Initialised(){
	std::cout << "Initialised" << std::endl;
	std::cout << "Going to Homed" << std::endl;
}

/** Homed
*/
void MySequence::Homed(){
	std::cout << "Homed" << std::endl;
	//if the sequence exist use it.
	MyBlockingSubSequence* subSequence = dynamic_cast<MyBlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("BlockingSubSequence"));
	if(!subSequence){
		//callerThread for Blocking Sub Sequence is the same as is in this Sequence.
		subSequence = new MyBlockingSubSequence("BlockingSubSequence", callerThread);
	}
	//In this case we will wait for the returning of the subSequence.run() method
	while(subSequence->getState() != eeros::sequencer::kSequenceFinished){
		subSequence->run();
	}
	std::cout << "Going to Move" << std::endl;
}

void MySequence::Move(){
	std::cout << "Move" << std::endl;
	std::cout << "Going to Next" << std::endl;
}


void MySequence::MoveException(){
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
		/*throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveException), 0,
			                                          errorHandlerA, true, false, "TestException");
													  */
		//case 2 and 4
		/*throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveException), 0,
			                                          errorHandlerA, false, false, "TestException");
													  */
		//next test
		throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&MySequence::MoveException),
					                                  static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised),
			                                          errorHandlerA, false, true, "TestException");
	}
}

void MySequence::MoveBlocking(){
	std::cout << "MoveBlocking" << std::endl;
	//if the sequence exist use it.
	MyBlockingSubSequence* subSequence = dynamic_cast<MyBlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("BlockingSubSequence"));
	if(!subSequence){
		//callerThread for Blocking Sub Sequence is the same as is in this Sequence.
		subSequence = new MyBlockingSubSequence("BlockingSubSequence", callerThread);
	}
	//MyBlockingSubSequence subSequence("BlockingSubSequence", callerThread);
	//In this case we will wait for the returning of the subSequence.run() method
	while(subSequence->getState() != eeros::sequencer::kSequenceFinished){
		subSequence->run();
	}
	std::cout << "Going to Next" << std::endl;
}

void MySequence::MoveNonBlocking(){
	std::cout << "MoveNonBlocking" << std::endl;
	MySequencer* subSequencer = 0;
	MyNonBlockingSubSequence* subSequence = dynamic_cast<MyNonBlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence"));
	if(!subSequence){
		//use pointer to leave the object in memory!!
		//without pointer the objetct will be destroyed.
		subSequencer = new MySequencer("SubSequencer");
		//callerThread for NonBlocking Sub Sequence is a new Sequencer
		//please take attention, if this Object looses scope, so it will be deleted!!
		//that's why you should use a pointer to allocate memory!!
		//else the pointer in the Executor runnables list of the Sequencer will point to nowhere!!
		//MyNonBlockingSubSequence is a Runnable!!
		subSequence = new MyNonBlockingSubSequence("NonBlockingSubSequence", *subSequencer);
	}

	//warten bis SubSequencer fertig ist
	bool suSequencerWasStarted = false;
	try{
		if(!subSequencer){
			//SubSequencer existiert schon und wurde hier nicht neu erzeugt
			subSequencer = dynamic_cast<MySequencer*>(eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer"));
			//For 5th case (-> note case 5 (in my Folder)).
			//set sequencerWasStarted = false to restart the sequencer
			//set sequencerWasStarted = true to not restart the sequencer and not waiting
			//suSequencerWasStarted = true;
			suSequencerWasStarted = false;
		}//else{
			//SubSequencer wurde neu erzeugt
			//sequencerWasStarted = false;
		//}
	
		if(!suSequencerWasStarted && subSequencer && subSequencer->getStatus() != kStopped){
			ExecutorService::waitForSequenceEnd(subSequencer);
		}

	}catch(char *str){

	}
	
	//now we start the Thread
	if(!suSequencerWasStarted){
		subSequencer->start();
	}

	std::cout << "Going to Next" << std::endl;
}

void MySequence::Moving(){
	std::cout << "Move" << std::endl;
	std::cout << "Going to Next" << std::endl;
}

/** Stopping
*/
void MySequence::Stopping(){
	std::cout << "Stopping" << std::endl;
	std::cout << "End of Sequence!" << this->getName() << std::endl;
	callerThread.stop();
}

void MySequence::WaitingForNonBlocking(){
	std::cout << "WaitingForNonBlocking" << std::endl;
	//Here we wait for the subsequencer Thread
	eeros::sequencer::Sequencer* seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	
	if(seq && seq->getStatus() != kStopped){
		ExecutorService::waitForSequenceEnd(seq);
	}

	//generate a fault for testing exception
	//seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer1");
}