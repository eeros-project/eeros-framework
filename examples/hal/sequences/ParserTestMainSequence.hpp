#ifndef CH_NTB_CB20TESTMAINSEQUENCE_HPP_
#define CH_NTB_CB20TESTMAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "../control/ParserTestControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/PeripheralOutput.hpp>

class ParserTestMainSequence : public eeros::sequencer::Sequence<void> {

public:
	ParserTestMainSequence(eeros::sequencer::Sequencer* sequencer, ParserTestControlSystem* controlSys, eeros::safety::SafetySystem* safetySys);
	
	virtual bool checkPreCondition();
	virtual void run();
	virtual void exit();
	
private:
	bool isTerminating();
	
	ParserTestControlSystem* controlSys;
	eeros::safety::SafetySystem* safetySys;
	
	
	
};
		
#endif // CH_NTB_CB20TESTMAINSEQUENCE_HPP_  
