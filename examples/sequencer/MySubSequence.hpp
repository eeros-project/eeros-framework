#pragma once

#include <eeros/sequencer/Sequence.hpp>

/** This examples shows how you can use a subsequence, if the main sequence will wait in a step of the terminaton of it
  *
  */
class MySubSequence : public eeros::sequencer::Sequence
{
public:
	MySubSequence(std::string name, TimeDomain* ptimeDomain);
	virtual ~MySubSequence(void);
private:
	void MoveToA();
	void MoveToB();
	void MoveToC();
	void Stopping();
};

