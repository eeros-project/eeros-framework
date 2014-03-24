#ifndef ORG_EEROS_CONTROL_SIGNALINTERFACE_HPP_
#define ORG_EEROS_CONTROL_SIGNALINTERFACE_HPP_

#include <string>
#include <sstream>
#include <vector>
#include <eeros/types.hpp>

namespace eeros {
	namespace control {
		
		enum {invalidSignalId = 0, startSignalId = 1, maxSignalId = 65535};
		
		class SignalInterface {
		public:
			
			virtual sigid_t getId() const = 0;
			
			virtual std::string getName() const = 0;
			
			virtual std::string getLabel() const = 0;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_SIGNALINTERFACE_HPP_ */
