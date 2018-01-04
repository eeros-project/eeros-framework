#ifndef ORG_EEROS_CORE_CONFIG_HPP_
#define ORG_EEROS_CORE_CONFIG_HPP_

#include <utility>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <map>
#include <functional>
#include <array>
#include <string.h>

namespace eeros {
	namespace config {

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

		/** \brief Configuration.
		*
		* This is the base class for a configuration to be saved to or loaded from disk.
		* A configuration might be useful to keep calibration values or setup data.
		*/
		class Config {
		public:
			/**
			* Creates a configuration
			* @param path Name of the configuration.
			*/
			Config(const char *path = nullptr);
			/**
			* Destructor, do not call manually.
			*/
			virtual ~Config();
			
			/**
			* Loads a default configuration, must be overwritten by a derived class.
			*/
			virtual void loadDefaults();
			/**
			* Saves a configuration, must be overwritten by a derived clas
			* @param path Name of the configuration.
			* @return true if successful
			*/
			virtual bool save(const char *path = nullptr) = 0;
			/**
			* Loads a configuration, must be overwritten by a derived clas
			* @param path Name of the configuration.
			* @return true if successful
			*/
			virtual bool load(const char *path = nullptr) = 0;
			
		protected:
			virtual void add(const char *name, int &value);
			virtual void add(const char *name, double &value);
			virtual void add(const char *name, std::size_t length, int *start, int *end, int default_value = -1);
			virtual void add(const char *name, std::size_t length, double *start, double *end, double default_value = NAN);
			virtual void add(const char *name, std::string &value);

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
}

#endif // ORG_EEROS_CORE_CONFIG_HPP_
