/*
 * Block1i.hpp
 *
 *  Created on: 15.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_BLOCK1I_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I_HPP_

#include "Block.hpp"
#include "Input.hpp"

class Block1i: public Block
{
public:
	Block1i();
	virtual ~Block1i();

	Input in;
};

#endif /* ORG_EEROS_CONTROL_BLOCK1I_HPP_ */
