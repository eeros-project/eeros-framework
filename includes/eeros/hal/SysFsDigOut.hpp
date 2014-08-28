#ifndef ORG_EEROS_HAL_SYSFSDIGOUT_HPP_
#define ORG_EEROS_HAL_SYSFSDIGOUT_HPP_

#include <eeros/hal/PeripheralOutput.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class SysFsDigOut : public PeripheralOutput<bool> {
		public:
			SysFsDigOut(std::string id, unsigned int gpio, bool inverted = false);
			~SysFsDigOut();
			virtual bool get();
			virtual void set(bool value);
			
		private:
			bool inverted;
			std::string basePath;
			std::fstream valueFile;
		};

	};
};

#endif /* ORG_EEROS_HAL_SYSFSDIGOUT_HPP_ */
