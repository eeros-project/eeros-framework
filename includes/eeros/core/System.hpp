#ifndef ORG_EEROS_CORE_SYSTEM_HPP_
#define ORG_EEROS_CORE_SYSTEM_HPP_

#include <stdint.h>

namespace eeros {

	class System {
	public:
		System();
		virtual ~System();

		static double getTime();
		static uint64_t getTimeNs();
		
	private:
		static uint64_t timeoffset;
	};

};

#endif /* ORG_EEROS_CORE_SYSTEM_HPP_ */
