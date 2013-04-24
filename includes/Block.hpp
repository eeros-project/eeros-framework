/*
 * Block.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_BLOCK_HPP_
#define ORG_EEROS_CONTROL_BLOCK_HPP_

class Block {
public:
	Block();
	virtual ~Block();
	virtual void run();
};

#endif /* ORG_EEROS_CONTROL_BLOCK_HPP_ */
