#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCER_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCER_HPP_

#include <eeros/sequencer/Sequence.hpp>

class MainSequence : public eeros::sequencer::Sequence {

public:
	MainSequence(std::string name);
	virtual ~MainSequence();
	
	virtual void init();
	virtual void exit();
	
private:
	int var;
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_MAINSEQUENCER_HPP_ 
