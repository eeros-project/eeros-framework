#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCE_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCE_HPP_

#include <string>

#include <eeros/sequencer/Sequence.hpp>

/** This is a example, how the user should work with this sequence.
  * 
  */
class eeros::sequencer::Sequencer;

class MySequence : public eeros::sequencer::Sequence
{
public:
	MySequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~MySequence(void);

	/** In this method you can fill for ex. all the methods you want to call, according the inserted order
	 */
	virtual void fillCallBacks();
	void fillVersion1();
	void fillVersion2();
	void fillVersion3();
	void fillVersion4();
	void fillVersion5();


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
	void MoveException();

	void MoveBlocking();
	void MoveNonBlocking();

	/** Moving waits until the Move is completed
	*/
	void Moving();

	void Waiting();
	void WaitingForNonBlocking();

	/** Stopping
	*/
	void Stopping();
};

#endif