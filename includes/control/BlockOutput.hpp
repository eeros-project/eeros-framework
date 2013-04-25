/*
 * BlockOutput.hpp
 *
 *  Created on: 24.04.2013
 *      Author: Martin Zueger
 */



#ifndef ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
#define ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP

#include "control/Block1i.hpp"


class BlockOutput : public Block1i
{

public:
	BlockOutput();
	virtual ~BlockOutput();

private:
	std::string identifier;
	double scale;
	double offset;
	double min;
	double max;
};

#endif // ORG_EEROS_CONTROL_BLOCKOUTPUT_HPP
