#ifndef ORG_EEROS_CONTROL_REALSIGNALINPUT_HPP_
#define ORG_EEROS_CONTROL_REALSIGNALINPUT_HPP_

#include <string>
#include <eeros/types.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
	namespace control {

		class RealSignalInput : public Input {
		public:
			virtual double getValue();
			virtual double getValue(int index);
			virtual sigdim_t getDimension();
			virtual uint64_t getTimestamp();
			virtual uint64_t getTimestamp(int index);	
			virtual std::string getName();
			virtual std::string getName(int index);
			virtual std::string getUnit();
			virtual std::string getUnit(int index);
			virtual std::string getCoordinateSystem();
			virtual std::string getCoordinateSystem(int index);
		};

	};
};

#endif /* ORG_EEROS_CONTROL_REALSIGNALINPUT_HPP_ */
