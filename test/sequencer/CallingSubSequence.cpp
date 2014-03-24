#include "CallingSubSequence.hpp"
#include "BlockingSubSequence.hpp"
#include "MySequencer.hpp"

#include <cppunit/extensions/HelperMacros.h>

CallingSubSequence::CallingSubSequence(std::string name, eeros::sequencer::Sequencer& caller)
	: MySequence(name, caller){
}

CallingSubSequence::~CallingSubSequence(){
}

void CallingSubSequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::move));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::callSubSequence));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::stopping));
}

void CallingSubSequence::callSubSequence(){
	//if the sequence exist use it.
	BlockingSubSequence* subSequence = dynamic_cast<BlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("BlockingSubSequence"));
	if(!subSequence){
		//callerThread for Blocking Sub Sequence is the same as is in this Sequence.
		subSequence = new BlockingSubSequence("BlockingSubSequence", callerThread);
	}
	//MyBlockingSubSequence subSequence("BlockingSubSequence", callerThread);
	//In this case we will wait for the returning of the subSequence.run() method
	while(subSequence->getState() != eeros::sequencer::kSequenceFinished){
		subSequence->run();
	}
	CPPUNIT_ASSERT(subSequence->getCalledMethode().compare("MoveToA MoveToB MoveToC ") == 0);
	calledMethode.append(subSequence->getCalledMethode());
}

/** Stopping
*/
void CallingSubSequence::stopping(){
	
	//stop the thread
	callerThread.stop();
	calledMethode.append("Stopping");
}