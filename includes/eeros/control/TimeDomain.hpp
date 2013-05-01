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
#include <eeros/core/Runnable.hpp>
#include <eeros/control/Block.hpp>

class TimeDomain : public Runnable
{
public:
	TimeDomain();
	TimeDomain(int divisor);
	virtual void addBlock(Block* block);
	virtual void sortBlocks();
	virtual void run();
	
	static void setMaster(TimeDomain* masterTimeDomain);
	
private:
	std::string name;
	int divisor;
	std::list<Block*> blocks;
	
	static TimeDomain* masterTimeDomain;
	static std::list<TimeDomain*> timeDomains;
};

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
