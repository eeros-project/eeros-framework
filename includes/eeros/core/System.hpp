#ifndef ORG_EEROS_CORE_SYSTEM_HPP_
#define ORG_EEROS_CORE_SYSTEM_HPP_

#include <stdint.h>

namespace eeros {

	static bool rosTimeIsUsed __attribute__((unused)) = false;
		
	class System {
	public:
		static double getClockResolution();
		static double getTime();
		static uint64_t getTimeNs();
		
#ifdef USE_ROS
		static void useRosTime();	
#endif
		
	private:
		System();
	};

};

#endif /* ORG_EEROS_CORE_SYSTEM_HPP_ */
