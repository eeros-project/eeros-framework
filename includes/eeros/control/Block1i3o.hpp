#ifndef ORG_EEROS_CONTROL_BLOCK1I3O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I3O_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

class Block1i3o: public Block {
public:
	Block1i3o(sigdim_t dim = 1);
	RealSignalInput& getIn();
	RealSignalOutput& getOut1();
	RealSignalOutput& getOut2();
	RealSignalOutput& getOut3();
	
protected:
	RealSignalInput in;
	RealSignalOutput out1;
	RealSignalOutput out2;
	RealSignalOutput out3;
};

#endif /* ORG_EEROS_CONTROL_BLOCK1I3O_HPP_ */
