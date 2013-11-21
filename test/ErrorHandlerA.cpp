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


#include "ErrorHandlerA.hpp"
#include "ErrorHandlerB.hpp"
#include "SequenceMoveException1_3.hpp"

#include <cppunit/extensions/HelperMacros.h>

ErrorHandlerA::ErrorHandlerA(std::string name, MySequence* s, bool callB)
	: eeros::sequencer::ErrorHandler(name), seq(s), callErrorHandlerB(callB){
}


ErrorHandlerA::~ErrorHandlerA(void){
}

void ErrorHandlerA::run(){
	Reset();
	if(callErrorHandlerB){
		Referencing();
	}
	RestartSequence();
	if(callErrorHandlerB){
		CPPUNIT_ASSERT(getCalledMethode().compare("Reset Referencing ResetB ReferencingB Restart ") == 0);
	}else{
		CPPUNIT_ASSERT(getCalledMethode().compare("Reset Restart ") == 0);
	}
	seq->calledMethode.append(getCalledMethode());
}

void ErrorHandlerA::Reset(){
	calledMethode.append("Reset ");
}

void ErrorHandlerA::Referencing(){
	calledMethode.append("Referencing ");
	ErrorHandlerB* errorHandlerB = dynamic_cast<ErrorHandlerB*>(eeros::sequencer::ErrorHandler::getErrorHandler("ErrorHandlerB"));
	if(!errorHandlerB){
		errorHandlerB = new ErrorHandlerB("ErrorHandlerB", seq);
	}
	errorHandlerB->run();
	calledMethode.append(errorHandlerB->getCalledMethode());
	
	delete errorHandlerB;
}

void ErrorHandlerA::RestartSequence(){
	calledMethode.append("Restart ");
}

std::string ErrorHandlerA::getCalledMethode(){
	return calledMethode;
}