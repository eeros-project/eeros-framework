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
	
	virtual sigmajorid_t getMajorId() const;
	virtual sigid_t getSignalId(sigindex_t index) const;
	virtual sigdim_t getDimension() const;
	virtual sigtype_t getType() const;
	virtual std::string getLabel() const;
	virtual std::string getLabel(sigindex_t index) const;
	
	static std::list<Signal*>* getSignalList();
	static Signal* getSignalById(sigid_t id);

protected:
	sigmajorid_t majorId; /**< unique signal id */
	sigdim_t dimension; /**< number of elements in this signal */

	static std::list<Signal*> signalList;
	static uint16_t signalCounter;
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
