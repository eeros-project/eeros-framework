#include "MyErrorHandlerA.hpp"
#include "MyErrorHandlerB.hpp"

//TODELETE
#include <iostream>

MyErrorHandlerA::MyErrorHandlerA(std::string name)
	: eeros::sequencer::ErrorHandler(name){
}


MyErrorHandlerA::~MyErrorHandlerA(void){
}

void MyErrorHandlerA::run(){
	Reset();
	Referencing();
	RestartSequence();
}

void MyErrorHandlerA::Reset(){
	std::cout << "MyErrorHandlerA::Reset" << std::endl;
	std::cout << "Going to Next" << std::endl;
}

void MyErrorHandlerA::Referencing(){
	std::cout << "MyErrorHandlerA::Referencing" << std::endl;
	std::cout << "Going to Next" << std::endl;
	MyErrorHandlerB* errorHandlerB = dynamic_cast<MyErrorHandlerB*>(eeros::sequencer::ErrorHandler::getErrorHandler("MyErrorHandlerB"));
	if(!errorHandlerB){
		errorHandlerB = new MyErrorHandlerB("MyErrorHandlerB");
	}
	errorHandlerB->run();
}

void MyErrorHandlerA::RestartSequence(){
	std::cout << "MyErrorHandlerA::RestartSequence" << std::endl;
	std::cout << "Going to Next" << std::endl;
}
