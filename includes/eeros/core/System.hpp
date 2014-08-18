#ifndef ORG_EEROS_CORE_SYSTEM_HPP_
#define ORG_EEROS_CORE_SYSTEM_HPP_

#include <stdint.h>

namespace eeros {

	class System {
	public:
		static double getClockResolution();
		static double getTime();
		static uint64_t getTimeNs();
		
	private:
		System();
	};

};

#endif /* ORG_EEROS_CORE_SYSTEM_HPP_ */
