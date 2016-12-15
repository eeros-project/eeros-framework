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
			virtual void createLogicObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber);
			virtual void createRealObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber, double scale, double offset, double rangeMin, double rangeMax, std::string unit);
			virtual void parseChannelProperties(ucl::Ucl chanObj, std::string *chanType, std::string *sigId, double *scale, double *offset, double *rangeMin, double *rangeMax, std::string *chanUnit);
										
			void calcScale(ucl::Ucl obj, double *scale, double *offset, double *rangeMin, double *rangeMax);
		};
	};
};

#endif /* ORG_EEROS_HAL_JSONPARSER_HPP_ */