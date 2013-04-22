/*
 * Block1i.hpp
 *
 *  Created on: 15.04.2013
 *      Author: zueger1
 */

#ifndef BLOCK1I_HPP_
#define BLOCK1I_HPP_

#include "Block.hpp"
#include "Input.hpp"

class Block1i: public Block
{
public:
	Block1i();
	virtual ~Block1i();

	Input in;
};

#endif /* BLOCK1I_HPP_ */
