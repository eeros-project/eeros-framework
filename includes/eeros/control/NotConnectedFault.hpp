#ifndef ORG_EEROS_CONTROL_NOTCONNECTEDFAULT_HPP
#define ORG_EEROS_CONTROL_NOTCONNECTEDFAULT_HPP

#include <eeros/core/Fault.hpp>

namespace eeros {
	namespace control {
		class NotConnectedFault : public eeros::Fault {

		public:
			NotConnectedFault();
			explicit NotConnectedFault(std::string m);
			virtual ~NotConnectedFault() throw();
		};
	};
};

#endif // ORG_EEROS_CONTROL_NOTCONNECTEDFAULT_HPP
