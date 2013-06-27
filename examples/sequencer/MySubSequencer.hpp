#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYSUBSEQUENCER_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYSUBSEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>

class MySubSequencer : public eeros::sequencer::Sequencer{
public:
	MySubSequencer(std::string name);
};

#endif