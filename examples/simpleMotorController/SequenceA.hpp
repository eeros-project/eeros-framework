#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_

#include <eeros/sequencer/Sequence.hpp>

class SequenceA : public eeros::sequencer::Sequence {

public:
	SequenceA(std::string name, double angle);
	virtual ~SequenceA();
	
	virtual bool checkPreCondition();
	virtual bool checkPostCondition();
	
	virtual void init();
	virtual void exit();
	
private:
	double angle;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_ 
