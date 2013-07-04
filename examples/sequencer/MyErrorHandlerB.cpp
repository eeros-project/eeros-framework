#include "MyErrorHandlerB.hpp"

//TODELETE
#include <iostream>

MyErrorHandlerB::MyErrorHandlerB(std::string name)
	: eeros::sequencer::ErrorHandler(name){
}


MyErrorHandlerB::~MyErrorHandlerB(void){
}

void MyErrorHandlerB::run(){
	Reset();
	Referencing();
	RestartSequence();
}

void MyErrorHandlerB::Reset(){
	std::cout << "MyErrorHandlerB::Reset" << std::endl;
	std::cout << "Going to Next" << std::endl;
}

void MyErrorHandlerB::Referencing(){
	std::cout << "MyErrorHandlerB::Referencing" << std::endl;
	std::cout << "Going to Next" << std::endl;
}

void MyErrorHandlerB::RestartSequence(){
	std::cout << "MyErrorHandlerB::RestartSequence" << std::endl;
	std::cout << "Going to Next" << std::endl;
}
