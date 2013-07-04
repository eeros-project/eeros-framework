#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYNONBLOCKINGSUBSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYNONBLOCKINGSUBSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>

/** This examples shows how you can use a subsequence, if the main sequence will wait in a step of their terminaton
  *
  */
class MyNonBlockingSubSequence : public eeros::sequencer::Sequence
{
public:
	MyNonBlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~MyNonBlockingSubSequence(void);
	virtual void fillCallBacks();
	void fillVersion1();
	void fillVersion2();
private:
	void MoveToAA();
	void MoveToBB();
	void MoveToCC();
	void Stopping();
	void MoveToException();
};

#endif
