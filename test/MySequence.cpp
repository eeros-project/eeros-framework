#include "MySequence.hpp"

MySequence::MySequence(std::string name, eeros::sequencer::Sequencer& caller)
	: eeros::sequencer::Sequence(name, caller){
	callerThread.addRunnable(this);
}

void MySequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::move));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::stopping));
}

/** Initialisation
 */
void MySequence::init(){
	calledMethode.append("Init ");
}

/** Initialising
	*/
void MySequence::initialising(){
	calledMethode.append("Initialising ");
}

/** Initialised
	*/
void MySequence::initialised(){
	calledMethode.append("Initialised ");
}

/** Homed
*/
void MySequence::homed(){
	calledMethode.append("Homed ");
}

/** Move
*/
void MySequence::move(){
	calledMethode.append("Move ");
}

/** Stopping
*/
void MySequence::stopping(){
	calledMethode.append("Stopping");
	//stop the thread
	callerThread.stop();
}

std::string MySequence::getCalledMethode(){
	return calledMethode;
}