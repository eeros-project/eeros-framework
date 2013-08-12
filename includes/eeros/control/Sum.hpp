#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

#include <vector>

class Sum : public Block {
public:
	Sum(uint32_t nofInputs = 2, sigdim_t dim = 1);

	virtual void run();
	
	virtual RealSignalInput& getIn(uint32_t input = 0);
	virtual RealSignalOutput& getOut();
	
	
	virtual void negateInput(uint32_t input);

protected:
	std::vector<RealSignalInput> in;
	RealSignalOutput out;
	
private:
	std::vector<bool> negated;
};

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
