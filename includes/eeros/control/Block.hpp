/*
 * Block.hpp
 *
 *  Created on: 11.04.2013
 *      Author: Martin Zueger
 */

#ifndef ORG_EEROS_CONTROL_BLOCK_HPP_
#define ORG_EEROS_CONTROL_BLOCK_HPP_

#include <string>
#include <eeros/core/Runnable.hpp>

class Block : public Runnable {
public:
//	Block();
//	virtual ~Block();
//	virtual void run()  = 0;

private:
	std::string name;
};

#endif /* ORG_EEROS_CONTROL_BLOCK_HPP_ */
