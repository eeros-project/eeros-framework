#include <eeros/control/TransitionBlock.hpp>

using namespace eeros;
using namespace eeros::control;


TransitionBlock::TransitionBlock() :
	runnableA(this, &TransitionBlock::runA),
	runnableB(this, &TransitionBlock::runB)
{
	
}

void TransitionBlock::setName(std::string name) { this->name = name; }
std::string TransitionBlock::getName() { return name; }

Runnable* TransitionBlock::getRunnableA() { return &runnableA; }
Runnable* TransitionBlock::getRunnableB() { return &runnableB; }

void TransitionBlock::runA() { };
void TransitionBlock::runB() { }
