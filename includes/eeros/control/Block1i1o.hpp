/*
 * Block1i1o.hpp
 *
 *  Created on: 01.05.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_BLOCK1I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

class Block1i1o: public Block
{
public:
	Block1i1o();
	virtual ~Block1i1o();

	Input in;
	Output* out;
};

#endif /* ORG_EEROS_CONTROL_BLOCK1I1O_HPP_ */
