#ifndef ORG_EEROS_CORE_CONFIG_HPP_
#define ORG_EEROS_CORE_CONFIG_HPP_

#include <eeros/math/Matrix.hpp>

#include <map>
#include <functional>
#include <array>
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
		virtual bool save(const char *path = nullptr) = 0;
		virtual bool load(const char *path = nullptr) = 0;
		
	protected:
		virtual void add(const char *name, int &value);
		virtual void add(const char *name, double &value);
		virtual void add(const char *name, std::size_t length, int *start, int *end, int default_value = -1);
		virtual void add(const char *name, std::size_t length, double *start, double *end, double default_value = NAN);

		template < typename T, std::size_t N >
		void add(const char *name, std::array<T,N> &value);
		
		const char *path;
		std::map<const char*, ConfigPropertyAccessor, CharPtrCompare> properties;
	};


	template < typename T, std::size_t N >
	void Config::add(const char *name, std::array<T,N> &value) {
		add(name, N, value.begin(), value.end());
	}

}

#endif // ORG_EEROS_CORE_CONFIG_HPP_
