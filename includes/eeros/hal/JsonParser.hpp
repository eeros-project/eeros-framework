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
		};
	};
};

#endif /* ORG_EEROS_HAL_JSONPARSER_HPP_ */