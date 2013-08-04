#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <list>
#include <string>
#include <eeros/types.hpp>

enum { kSignalTypeUnknown = 0, kSignalIdInvalid = -1 };

class Signal {
public:
	Signal(sigdim_t dim);
	virtual ~Signal();
	
	virtual sigid_t getSignalId() const;
	virtual sigdim_t getLength() const;
	virtual sigtype_t getType() const;
	virtual std::string getLabel() const;
	virtual std::string getLabel(int index) const;
	
	static std::list<Signal*>* getSignalList();
	static Signal* getSignalById(uint32_t id);

protected:
	sigid_t id; /**< unique signal id */
	sigdim_t length; /**< number of elements in this signal */

	static std::list<Signal*> signalList;
	static uint32_t signalCounter;
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
