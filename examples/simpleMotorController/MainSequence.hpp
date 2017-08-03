#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MySafetyProperties.hpp"
#include "MyControlSystem.hpp"

using namespace eeros::sequencer;
using namespace eeros::safety;
using namespace eeros::logger;

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& sequencer, SafetySystem& safetySys, MySafetyProperties& safetyProp, MyControlSystem& controlSys, double angle);
	int action();
	
private:
	bool isTerminating();
	double angle;
	SafetySystem& safetySys;
	MyControlSystem& controlSys;
	MySafetyProperties& safetyProp;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCE_HPP_ 
