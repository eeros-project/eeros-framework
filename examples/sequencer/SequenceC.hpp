#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEC_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEC_HPP_

#include <eeros/sequencer/Sequence.hpp>

class SequenceC : public eeros::sequencer::Sequence {

public:
	SequenceC(std::string name);
	virtual ~SequenceC();
	
	virtual void init();
	virtual void exit();
};

#endif // ORG_EEROS_EXAMPLES_SEQUENCER_SEQUENCEC_HPP_ 
