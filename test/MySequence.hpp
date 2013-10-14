#ifndef ORG_EEROS_TEST_MYSEQUENCE_HPP_
#define ORG_EEROS_TEST_MYSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>

class MySequence: public eeros::sequencer::Sequence{
private:
	std::string calledMethode;
public:
	MySequence(std::string name, eeros::sequencer::Sequencer& caller);
	
	virtual void fillCallBacks();
	
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

	/** Stopping
	*/
	void Stopping();
	
	std::string getCalledMethode();
};

#endif