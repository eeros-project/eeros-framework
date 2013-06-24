#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/BlockOutput.hpp>
#include <eeros/control/Step.hpp>

#include <eeros/sequencer/Sequence.hpp>

//TODELETE
#include <iostream>

#include "MySequence.hpp"
#include "MySubSequence.hpp"

MySequence::MySequence(std::string name, TimeDomain* ptimeDomain):
eeros::sequencer::Sequence(name, ptimeDomain),
currentSubSequence(0){
	//next( (static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init) ));
	next( (eeros::sequencer::Sequence::method)(&MySequence::Init) );
}


MySequence::~MySequence(void){
	deleteAllSubSequences();
}

void MySequence::Init(){
	std::cout << "Init" << std::endl;
	std::cout << "Going to Initialising" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySequence::Initialising));
}

/** Initialising
*/
void MySequence::Initialising(){
	std::cout << "Initialising" << std::endl;
	std::cout << "Going to Initialised" << std::endl;
	//Create blocks for control
	Step* step = new Step(1.0, 5.0, 0.0);
	Gain* gain = new Gain(10);
	BlockOutput* output = new BlockOutput();
	gain->getIn().connect(step->getOut());
	output->getIn().connect(gain->getOut());

	//add the blocks to the time domain
	timeDomain->addBlock(step);
	timeDomain->addBlock(gain);
	timeDomain->addBlock(output);

	next((eeros::sequencer::Sequence::method)(&MySequence::Initialised));
}

/** Initialised
*/
void MySequence::Initialised(){
	std::cout << "Initialised" << std::endl;
	std::cout << "Going to Homed" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySequence::Homed));
}

/** Homed
*/
void MySequence::Homed(){
	std::cout << "Homed" << std::endl;
	std::cout << "Going to Move" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySequence::Move));
}

/** Move
*/
//For examples execution variants:
//#define WAITING
#undef WAITING

void MySequence::Move(){
	std::cout << "Move" << std::endl;

#ifdef WAITING
	//variant: waits until the subsequence has finished
	if(!currentSubSequence){
		currentSubSequence = new MySubSequence("SubSequence", timeDomain);
		currentSubSequence->start();
	}
	if(currentSubSequence && currentSubSequence->getStatus() == kStopped){
		currentSubSequence = 0;
		std::cout << "Going to Moving" << std::endl;
		next((eeros::sequencer::Sequence::method)(&MySequence::Moving));
	}

#else if

	//variant: wait in an other step for example in Step Waiting for termination
	if(!currentSubSequence){
		currentSubSequence = new MySubSequence("SubSequence", timeDomain);
		currentSubSequence->start();
		addSubSequence(currentSubSequence);
		currentSubSequence = 0;
		std::cout << "Going to Moving" << std::endl;
		next((eeros::sequencer::Sequence::method)(&MySequence::Moving));
	}
#endif
}

/** Moving waits until the Move is completed
*/
void MySequence::Moving(){
	std::cout << "Moving" << std::endl;
	std::cout << "Going to Stopping" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySequence::Waiting));
}

/** Moving waits until the Move is completed
*/
void MySequence::Waiting(){
	std::cout << "Waiting for SubSequence" << std::endl;
	//Waiting for SubSequence to terminate
	//get SubSequende in list
	Sequence* waitForSeq = findSequence("SubSequence");
	ExecutorService::waitForSequenceEnd(waitForSeq);
	std::cout << "Going to Stopping" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySequence::Stopping));
}

/** Stopping
*/
void MySequence::Stopping(){
	std::cout << "Stopping" << std::endl;
	std::cout << "End of Sequence!" << std::endl;
	stop();
}
