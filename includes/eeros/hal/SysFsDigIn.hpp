#ifndef ORG_EEROS_HAL_SYSFSDIGIN_HPP_
#define ORG_EEROS_HAL_SYSFSDIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <fstream>
#include <string>

namespace eeros {
	namespace hal {
		class SysFsDigIn : public Input<bool> {
		public:
			SysFsDigIn(std::string id, void* libHandle, unsigned int gpio, bool inverted = false);
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
