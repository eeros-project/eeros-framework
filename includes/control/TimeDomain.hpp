/*
 * TimeDomain.hpp
 *
 *  Created on: 24.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROLTIMEDOMAIN_HPP
#define ORG_EEROS_CONTROLTIMEDOMAIN_HPP

#include <list>
#include <string>
#include "control/Block.hpp"

class TimeDomain
{
public:
	void run();
	void addBlock(Block* block);
	void sortBlocks();
	
private:
	std::string name;
	int divisor;
	
	std::list<Block*> blocks;
};

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
