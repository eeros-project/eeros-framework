#ifndef ORG_EEROS_HAL_JSONPARSER_HPP_
#define ORG_EEROS_HAL_JSONPARSER_HPP_

#include <ucl++.h>

namespace eeros {
	namespace hal {
		class JsonParser
		{
		public:
			JsonParser();
			JsonParser(std::string filePath);
			virtual void createHalObjects(std::map<std::string, void*> lib);
		private:
			ucl::Ucl halRootObj;
			void createLogicObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber);
			void createRealObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber, double scale, double offset);
			template<typename T> 
			T getType(std::string type);
		};
	};
};

#endif /* ORG_EEROS_HAL_JSONPARSER_HPP_ */