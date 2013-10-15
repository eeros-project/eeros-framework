#ifndef ORG_EEROS_TEST_BLOCKINGSUBSEQUENCE_HPP_
#define ORG_EEROS_TEST_BLOCKINGSUBSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>

/** This examples shows how you can use a subsequence, if the main sequence will wait in a step of their terminaton
  *
  */
class BlockingSubSequence : public eeros::sequencer::Sequence
{
private:
	std::string calledMethode;
public:
	BlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~BlockingSubSequence(void);
	virtual void fillCallBacks();
	std::string getCalledMethode();

private:
	void MoveToA();
	void MoveToB();
	void MoveToC();	
};

#endif
