#include "MySequence.hpp"

MySequence::MySequence(std::string name, eeros::sequencer::Sequencer& caller)
	: eeros::sequencer::Sequence(name, caller){
	callerThread.addRunnable(this);
}

void MySequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Move));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&MySequence::Stopping));
}

/** Initialisation
 */
void MySequence::Init(){
	calledMethode.append("Init");
}

/** Initialising
	*/
void MySequence::Initialising(){
	calledMethode.append("Initialising");
}

/** Initialised
	*/
void MySequence::Initialised(){
	calledMethode.append("Initialised");
}

/** Homed
*/
void MySequence::Homed(){
	calledMethode.append("Homed");
}

/** Move
*/
void MySequence::Move(){
	calledMethode.append("Move");
}

/** Stopping
*/
void MySequence::Stopping(){
	calledMethode.append("Stopping");
	callerThread.stop();
}

std::string MySequence::getCalledMethode(){
	return calledMethode;
}