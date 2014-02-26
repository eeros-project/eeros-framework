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


#include "CallingNonBlockingSequence_ErrorHandler.hpp"
#include "ErrorHandlerA.hpp"

#include <eeros/sequencer/SequenceException.hpp>

int CallingNonBlockingSequence_ErrorHandler::i = 0;

CallingNonBlockingSequence_ErrorHandler::CallingNonBlockingSequence_ErrorHandler(std::string name, eeros::sequencer::Sequencer& caller, bool restart)
	: CallingNonBlockingSubSequence(name, caller, restart, false){
	//callerThread.addRunnable(this);
}

CallingNonBlockingSequence_ErrorHandler::~CallingNonBlockingSequence_ErrorHandler(){
}

void CallingNonBlockingSequence_ErrorHandler::fillCallBacks(){
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::init));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialising));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::initialised));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSubSequence::callSubSequence));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::homed));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSequence_ErrorHandler::moveException));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSubSequence::wait));
	addCallBack(static_cast<eeros::sequencer::Sequence::method>(&CallingSubSequence::stopping));
}

void CallingNonBlockingSequence_ErrorHandler::moveException(){
	calledMethode.append("MoveException ");
	
	//Please take attention the exception should only be thrown once, else the sequencer thread will never end.
	//The sequence CallingNonBlockingSequence_ErrorHandler is restarted at next run() call of the base sequence, called by the executor
	if(i == 0) {
		i++;
		ErrorHandlerA* errorHandlerA = dynamic_cast<ErrorHandlerA*>(eeros::sequencer::ErrorHandler::getErrorHandler("ErrorHandlerA"));
		if(!errorHandlerA){
			errorHandlerA = new ErrorHandlerA("ErrorHandlerA", this, false);
		}
		//case 2a (except the call in ErrorHandlerA to ErrorHandlerB it is also Casse 4a)
		//After this exception the sequence is continues in this method
		throw new eeros::sequencer::SequenceException(this, static_cast<eeros::sequencer::Sequence::method>(&CallingNonBlockingSequence_ErrorHandler::moveException), 0,
			                                      errorHandlerA, true, false, "Exception ErrorHandlerA");
	}
}