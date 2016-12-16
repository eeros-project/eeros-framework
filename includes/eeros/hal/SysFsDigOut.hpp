#ifndef ORG_EEROS_HAL_SYSFSDIGOUT_HPP_
#define ORG_EEROS_HAL_SYSFSDIGOUT_HPP_

#include <eeros/hal/Output.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class SysFsDigOut : public Output<bool> {
		public:
			SysFsDigOut(std::string id, void* libHandle, unsigned int gpio, bool inverted = false);
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
