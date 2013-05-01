/*
 * System.hpp
 *
 *  Created on: 22.04.2013
 *      Author: zueger1
 */

#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <stdint.h>

class System
{
public:
	System();
	virtual ~System();

	static double getTime();
	
private:
	static uint64_t timeoffset;
};

#endif /* SYSTEM_HPP_ */
