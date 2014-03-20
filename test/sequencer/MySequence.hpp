#ifndef ORG_EEROS_TEST_MYSEQUENCE_HPP_
#define ORG_EEROS_TEST_MYSEQUENCE_HPP_

#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>

class MySequence: public eeros::sequencer::Sequence{
	friend class ErrorHandlerA;
protected:
	std::string calledMethode;
public:
	MySequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~MySequence();
	
	virtual void fillCallBacks();
	
	/** Initialisation
	 */
	void init();

	/** Initialising
	 */
	void initialising();

	/** Initialised
	 */
	void initialised();

	/** Homed
	*/
	void homed();

	/** Move
	*/
	void move();

	/** Stopping
	*/
	void stopping();
	
	std::string getCalledMethode();
};

#endif