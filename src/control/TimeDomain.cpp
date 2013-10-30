#include <eeros/control/TimeDomain.hpp>

using namespace eeros::control;

TimeDomain::TimeDomain(){
}

void TimeDomain::run()
{
	for(std::list<Block*>::iterator i = blocks.begin(); i != blocks.end(); i++)
	{
		(*i)->run();
	}
}

void TimeDomain::addBlock(Block* block)
{
	blocks.push_back(block);
}

void TimeDomain::sortBlocks()
{
	// TODO
}