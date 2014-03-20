#ifndef ORG_EEROS_TEST_BLOCKINGSUBSEQUENCE_HPP_
#define ORG_EEROS_TEST_BLOCKINGSUBSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>

/** This examples shows how you can use a subsequence, if the main sequence will wait in a step of their terminaton
  * This sequence has not to be saved to the executer as runnable, because the superior sequnces will call the run method
  */
class BlockingSubSequence : public eeros::sequencer::Sequence
{
protected:
	std::string calledMethode;
public:
	BlockingSubSequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~BlockingSubSequence(void);
	virtual void fillCallBacks();
	std::string getCalledMethode();

protected:
	void MoveToA();
	void MoveToB();
	void MoveToC();	
};

#endif
