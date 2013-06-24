#pragma once

#include <string>

class MySubSequence;

/** This is a example, how the user should work with this sequence.
  *
  */
class MySequence : public eeros::sequencer::Sequence
{
public:
	MySequence(std::string name, TimeDomain* ptimeDomain);
	virtual ~MySequence(void);
private:		
	MySubSequence* currentSubSequence;
	/** Initialisation
	*/
	void Init();

	/** Initialising
	*/
	void Initialising();

	/** Initialised
	*/
	void Initialised();

	/** Homed
	*/
	void Homed();

	/** Move
	*/
	void Move();

	/** Moving waits until the Move is completed
	*/
	void Moving();

	/** Stopping
	*/
	void Stopping();
};

