#ifndef ORG_EEROS_CORE_SIMPLECONFIG_HPP_
#define ORG_EEROS_CORE_SIMPLECONFIG_HPP_

#include <eeros/core/Config.hpp>

namespace eeros {

	class SimpleConfig : public Config {
	public:
		SimpleConfig(const char *path = nullptr);
		virtual ~SimpleConfig();
		
		virtual void save(const char *path = nullptr);
		virtual void load(const char *path = nullptr);
		
		static constexpr int buffer_size = 256;
	};

};

#endif // ORG_EEROS_CORE_SIMPLECONFIG_HPP_
