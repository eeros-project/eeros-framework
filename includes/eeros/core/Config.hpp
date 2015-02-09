#ifndef ORG_EEROS_CORE_CONFIG_HPP_
#define ORG_EEROS_CORE_CONFIG_HPP_

#include <eeros/math/Matrix.hpp>

#include <map>
#include <functional>
#include <string.h>

namespace eeros {

	struct ConfigPropertyAccessor {
		std::function<int(const char *, char *, int)> set;
		std::function<int(const char *, const char *, int)> get;
	};

	struct CharPtrCompare
	{
		bool operator()(const char *first, const char  *second)
		{
			return strcmp(first, second) < 0;
		}
	};

	class Config {
	public:
		Config(const char *path = nullptr);
		virtual ~Config();
		
		virtual void loadDefaults();
		virtual void save(const char *path = nullptr) = 0;
		virtual void load(const char *path = nullptr) = 0;
		
	protected:
		virtual void add(const char *name, int &value);
		virtual void add(const char *name, double &value);
		
		const char *path;
		std::map<const char*, ConfigPropertyAccessor, CharPtrCompare> properties;
	};

};

#endif // ORG_EEROS_CORE_CONFIG_HPP_
