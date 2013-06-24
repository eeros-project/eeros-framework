#include <eeros/control/TimeDomain.hpp>

#include "MySubSequence.hpp"

//TODELETE
#include <iostream>

MySubSequence::MySubSequence(std::string name, TimeDomain* ptimeDomain):
eeros::sequencer::Sequence(name, ptimeDomain){
	next((eeros::sequencer::Sequence::method)(&MySubSequence::MoveToA));
}


MySubSequence::~MySubSequence(void)
{
}

void MySubSequence::MoveToA(){
	std::cout << "MoveToA" << std::endl;
	std::cout << "Going to MoveToB" << std::endl;
	timeDomain->run();
	next((eeros::sequencer::Sequence::method)(&MySubSequence::MoveToB));
}

void MySubSequence::MoveToB(){
	std::cout << "MoveToA" << std::endl;
	std::cout << "Going to MoveToC" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySubSequence::MoveToC));
}

void MySubSequence::MoveToC(){
	std::cout << "MoveToA" << std::endl;
	std::cout << "Going to Stopping" << std::endl;
	next((eeros::sequencer::Sequence::method)(&MySubSequence::Stopping));
}

void MySubSequence::Stopping(){
	std::cout << "Stopping" << std::endl;
	std::cout << "End of Sequence!" << std::endl;
	stop();
}
