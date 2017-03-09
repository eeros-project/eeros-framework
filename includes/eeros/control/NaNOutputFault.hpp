#ifndef ORG_EEROS_CONTROL_NANOUTPUTFAULT_HPP
#define ORG_EEROS_CONTROL_NANOUTPUTFAULT_HPP

#include <eeros/core/Fault.hpp>

namespace eeros {
	namespace control {
		class NaNOutputFault : public eeros::Fault {

		public:
			NaNOutputFault();
			explicit NaNOutputFault(std::string m);
			virtual ~NaNOutputFault() throw();
		};
	};
};

#endif // ORG_EEROS_CONTROL_NANOUTPUTFAULT_HPP
