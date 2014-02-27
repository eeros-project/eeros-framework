#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEB_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEB_HPP_

#include <eeros/sequencer/Sequence.hpp>

class SequenceB : public eeros::sequencer::Sequence {

public:
	SequenceB(std::string name);
	virtual ~SequenceB();
	
	virtual void init();
	virtual void exit();
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEB_HPP_ 
