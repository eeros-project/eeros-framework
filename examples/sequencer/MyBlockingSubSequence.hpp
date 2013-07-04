#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYBLOCKINGSUBSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYBLOCKINGSUBSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>

/** This examples shows how you can use a subsequence, if the main sequence will wait in a step of their terminaton
  *
  */
class MyBlockingSubSequence : public eeros::sequencer::Sequence
{
public:
	MyBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~MyBlockingSubSequence(void);
	virtual void fillCallBacks();
	void fillVersion1();
	void fillVersion2();

private:
	void MoveToA();
	void MoveToB();
	void MoveToC();
	void MoveToException();
	void Stopping();
};

#endif
