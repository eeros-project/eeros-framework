#ifndef ORG_EEROS_CONFIG_FILECONFIG_HPP_
#define ORG_EEROS_CONFIG_FILECONFIG_HPP_

#include <eeros/config/Config.hpp>

namespace eeros {
	namespace config {

		class FileConfig : public Config {
		public:
			FileConfig(const char *path = nullptr);
			virtual ~FileConfig();
			
			virtual bool save(const char *path = nullptr);
			virtual bool load(const char *path = nullptr);
			
			static constexpr int buffer_size = 256;
		};
	};
};

#endif // ORG_EEROS_CONFIG_FILECONFIG_HPP_
