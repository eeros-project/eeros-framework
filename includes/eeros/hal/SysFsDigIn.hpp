#ifndef ORG_EEROS_HAL_SYSFSDIGIN_HPP_
#define ORG_EEROS_HAL_SYSFSDIGIN_HPP_

#include <eeros/hal/PeripheralInput.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class SysFsDigIn : public PeripheralInput<bool> {
		public:
			SysFsDigIn(std::string id, unsigned int gpio, bool inverted = false);
			~SysFsDigIn();
			virtual bool get();
			
		private:
			bool inverted;
			std::string basePath;
			std::fstream valueFile;
		};

	};
};

#endif /* ORG_EEROS_HAL_SYSFSDIGIN_HPP_ */
