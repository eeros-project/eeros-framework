#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCER_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>

class MySequencer : public eeros::sequencer::Sequencer{
public:
	MySequencer(std::string name);
};

#endif