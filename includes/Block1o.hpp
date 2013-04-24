/*
 * Block1o.hpp
 *
 *  Created on: 15.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_BLOCK1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1O_HPP_

#include "Block.hpp"
#include "Output.hpp"

class Block1o: public Block
{
public:
	Block1o();
	virtual ~Block1o();

	Output* out;
};

#endif /* ORG_EEROS_CONTROL_BLOCK1O_HPP_ */
