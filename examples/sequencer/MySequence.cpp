//TODELETE
#include <iostream>

#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/Step.hpp>

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceException.hpp>

#include "MySequence.hpp"
#include "MySubSequencer.hpp"
#include "MyBlockingSubSequence.hpp"
#include "MyNonBlockingSubSequence.hpp"

MySequence::MySequence(std::string name, eeros::sequencer::Sequencer& caller)
	: eeros::sequencer::Sequence(name, caller){
		callerThread.addRunnable(this);
}


MySequence::~MySequence(void){
}

void MySequence::fillCallBacks(){
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
	subSequence->run();
	std::cout << "Going to Move" << std::endl;
}

void MySequence::Move(){
	std::cout << "Move" << std::endl;
	std::cout << "Going to Next" << std::endl;
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
	subSequence->run();
	std::cout << "Going to Next" << std::endl;
}

void MySequence::MoveNonBlocking(){
	std::cout << "MoveNonBlocking" << std::endl;
	//use pointer to leave the object in memory!!
	//without pointer the objetct will be destroyed.
	MySubSequencer* subSequencer = new MySubSequencer("SubSequencer");
	//callerThread for NonBlocking Sub Sequence is a new Sequencer
	//please take attention, if this Object looses scope, so it will be deleted!!
	//that's why you should use a pointer to allocate memory!!
	//else the pointer in the Executor runnables list of the Sequencer will point to nowhere!!
	//MyNonBlockingSubSequence is a Runnable!!
	MyNonBlockingSubSequence* subSequence = new MyNonBlockingSubSequence("NonBlockingSubSequence", *subSequencer);
	//now we start the Thread
	subSequencer->start();
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
}

eeros::sequencer::Sequence* MySequence::createSequence(std::string name, eeros::sequencer::Sequencer& caller){
	return new MySequence(name, caller);
}