#ifndef CH_NTB_SIMMAINSEQUENCE_HPP_
#define CH_NTB_SIMMAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/SimControlSystem.hpp"
#include "../safety/SimSafetyProperties.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/Output.hpp>

class SimMainSequence : public eeros::sequencer::Sequence<void> {

public:
	SimMainSequence(eeros::sequencer::Sequencer* sequencer, SimControlSystem* controlSys, eeros::safety::SafetySystem* safetySys, SimSafetyProperties* properties);
	
	virtual bool checkPreCondition();
	virtual void run();
	virtual void exit();
	
private:
	bool isTerminating();
	
	SimControlSystem* controlSys;
	eeros::safety::SafetySystem* safetySys;
	SimSafetyProperties* safetyProp;
	
	eeros::hal::HAL& hal = eeros::hal::HAL::instance();
	eeros::hal::Input<bool> &simIn_in1 = *hal.getLogicInput("in1");
	eeros::hal::Output<bool> &simIn_out1 = *hal.getLogicOutput("out1");
	
	eeros::hal::Output<double> &aOut0 = *hal.getRealOutput("aOut0");
// 	eeros::hal::Input<double> &aIn2 = *hal.getRealInput("aIn2");
// 	eeros::hal::Output<double> &aOutTest2 = *hal.getRealOutput("aOutTest2");
};
		
#endif // CH_NTB_SIMMAINSEQUENCE_HPP_  
