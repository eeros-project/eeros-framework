#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <list>
#include <string>
#include <eeros/types.hpp>

//Forward Declarations
class Signal;

class Signal {
public:
	Signal();
	virtual ~Signal();
	uint32_t getSignalId();
	virtual uint32_t getLength();
	virtual std::string getLabel();
	virtual std::string getLabel(int index);
	
	static std::list<Signal*>* getSignalList();

protected:
	uint32_t id; /**< unique signal id */
	uint32_t length; /**< number of elements in this signal */
	
	static std::list<Signal*> signalList;
	static uint32_t signalCounter;
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
