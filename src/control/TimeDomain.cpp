/*
 * TimeDomain.cpp
 *
 *  Created on: 24.04.2013
 *      Author: Martin Zueger
 */

#include "control/TimeDomain.hpp"

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