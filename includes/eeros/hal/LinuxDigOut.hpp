#ifndef ORG_EEROS_HAL_LINUXDIGOUT_HPP_
#define ORG_EEROS_HAL_LINUXDIGOUT_HPP_

#include <string>
#include <stdio.h>
#include <eeros/hal/PeripheralOutput.hpp>

namespace eeros {
	namespace hal {
		class LinuxDigOut : public PeripheralOutput<bool> {
		public:
			LinuxDigOut(std::string id, uint32_t gpioNr, bool inverted = false);
			~LinuxDigOut();	
			virtual bool get();
			virtual void set(bool value);
			
		private:
			int32_t valueFd;
			bool inverted;
		};

	};
};

#endif /* ORG_EEROS_HAL_LINUXDIGOUT_HPP_ */
