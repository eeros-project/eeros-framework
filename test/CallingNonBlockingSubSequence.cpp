/*
    The contents of this file are subject to the Mozilla Public License
    Version 1.1 (the "License"); you may not use this file except in
    compliance with the License. You may obtain a copy of the License at
    http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS IS"
    basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
    License for the specific language governing rights and limitations
    under the License.

    The Original Code is ______________________________________.

    The Initial Developer of the Original Code is ________________________.
    Portions created by ______________________ are Copyright (C) ______
    _______________________. All Rights Reserved.

    Contributor(s): ______________________________________.

    Alternatively, the contents of this file may be used under the terms
    of the _____ license (the  "[___] License"), in which case the
    provisions of [______] License are applicable instead of those
    above.  If you wish to allow use of your version of this file only
    under the terms of the [____] License and not to allow others to use
    your version of this file under the MPL, indicate your decision by
    deleting  the provisions above and replace  them with the notice and
    other provisions required by the [___] License.  If you do not delete
    the provisions above, a recipient may use your version of this file
    under either the MPL or the [___] License."
*/


#include "CallingNonBlockingSubSequence.hpp"
#include "MySequencer.hpp"
#include "NonBlockingSubSequence.hpp"

#include <eeros/core/Executor.hpp>

#include <cppunit/extensions/HelperMacros.h>


CallingNonBlockingSubSequence::CallingNonBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller)
	: CallingSubSequence(name, caller){
	//callerThread.addRunnable(this);
}

CallingNonBlockingSubSequence::~CallingNonBlockingSubSequence(){
}

void CallingNonBlockingSubSequence::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSubSequence::callSubSequence));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::move));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSubSequence::wait));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::stopping));
}

void CallingNonBlockingSubSequence::callSubSequence(){
	MySequencer* subSequencer = 0;
	NonBlockingSubSequence* subSequence = dynamic_cast<NonBlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence"));
	if(!subSequence){
		//use pointer to leave the object in memory!!
		//without pointer the objetct will be destroyed.
		subSequencer = new MySequencer("SubSequencer");
		//callerThread for NonBlocking Sub Sequence is a new Sequencer
		//please take attention, if this Object looses scope, so it will be deleted!!
		//that's why you should use a pointer to allocate memory!!
		//else the pointer in the Executor runnables list of the Sequencer will point to nowhere!!
		//MyNonBlockingSubSequence is a Runnable!!
		subSequence = new NonBlockingSubSequence("NonBlockingSubSequence", *subSequencer);
	}

	//warten bis SubSequencer fertig ist
	bool subSequencerWasStarted = false;
	try{
		if(!subSequencer){
			//SubSequencer existiert schon und wurde hier nicht neu erzeugt
			subSequencer = dynamic_cast<MySequencer*>(eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer"));
			//For 5th case (-> note case 5 (in my Folder)).
			//set sequencerWasStarted = false to restart the sequencer
			//set sequencerWasStarted = true to not restart the sequencer and not waiting
			//suSequencerWasStarted = true;
			subSequencerWasStarted = false;
		}//else{
			//SubSequencer wurde neu erzeugt
			//sequencerWasStarted = false;
		//}
	
		if(!subSequencerWasStarted && subSequencer && subSequencer->getStatus() != eeros::kStopped){
			eeros::ExecutorService::waitForSequenceEnd(subSequencer);
		}

	}catch(char *str){

	}
	
	//now we start the Thread
	if(!subSequencerWasStarted){
		subSequencer->start();
	}
}

void CallingNonBlockingSubSequence::wait(){
	//Here we wait for the subsequencer Thread
	eeros::sequencer::Sequencer* seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	
	if(seq && seq->getStatus() != eeros::kStopped){
		eeros::ExecutorService::waitForSequenceEnd(seq);
	}
	
	NonBlockingSubSequence* subSequence = dynamic_cast<NonBlockingSubSequence*>(eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence"));
	CPPUNIT_ASSERT(subSequence->getCalledMethode().compare("MoveToA MoveToB MoveToC Stop ") == 0);
	calledMethode.append(subSequence->getCalledMethode());
	calledMethode.append("Wait ");
	
	delete seq;
}
