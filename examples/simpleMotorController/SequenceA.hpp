#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"

class SequenceA : public eeros::sequencer::Sequence<void> {

public:
	SequenceA(std::string name, eeros::sequencer::Sequencer& sequencer, eeros::safety::SafetySystem& safetySys, MySafetyProperties& properties, MyControlSystem& controlSys, double angle);
	
	virtual bool checkPreCondition();
// 	virtual bool checkPostCondition();
	
	virtual void run();
	virtual void exit();
	
private:
	bool isTerminating();
	double angle;
	eeros::safety::SafetySystem& safetySys;
	MyControlSystem& controlSys;
	MySafetyProperties& safetyProp;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEA_HPP_ 
