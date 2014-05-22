#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>

class SequenceA : public eeros::sequencer::Sequence {

public:
	SequenceA(std::string name, eeros::safety::SafetySystem& safetySys, double angle);
	virtual ~SequenceA();
	
	virtual bool checkPreCondition();
	virtual bool checkPostCondition();
	
	virtual void init();
	virtual void exit();
	
private:
	double angle;
	eeros::safety::SafetySystem& safetySys;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_ 
