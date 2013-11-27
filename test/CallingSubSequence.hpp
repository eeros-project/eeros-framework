#ifndef ORG_EEROS_TEST_CALLINGSUBSEQUENCE_HPP_
#define ORG_EEROS_TEST_CALLINGSUBSEQUENCE_HPP_

#include "MySequence.hpp"
#include <eeros/sequencer/Sequencer.hpp>

class CallingSubSequence: public MySequence{
public:
	CallingSubSequence(std::string name, eeros::sequencer::Sequencer& caller);
	virtual ~CallingSubSequence();
	
	virtual void fillCallBacks();
	
	/** Call SubSequence
	 * */
	virtual void callSubSequence();
	
	
	/** Stopping
	*/
	void stopping();
};

#endif