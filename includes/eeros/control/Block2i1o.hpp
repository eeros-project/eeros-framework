#ifndef ORG_EEROS_CONTROL_BLOCK2I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK2I1O_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

class Block2i1o: public Block {
public:
	Block2i1o(sigdim_t dim = 1);
	RealSignalInput& getIn1();
	RealSignalInput& getIn2();
	RealSignalOutput& getOut();
	
protected:
	RealSignalInput in1;
	RealSignalInput in2;
	RealSignalOutput out;
};

#endif /* ORG_EEROS_CONTROL_BLOCK2I1O_HPP_ */
