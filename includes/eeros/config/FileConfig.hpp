#ifndef ORG_EEROS_CONFIG_FILECONFIG_HPP_
#define ORG_EEROS_CONFIG_FILECONFIG_HPP_

#include <eeros/config/Config.hpp>

namespace eeros {
	namespace config {

		/** \brief Configuration saved to and loaded from file.
		*
		* This allows for saving a configuration to a file on disk or loading it from there.
		* A configuration might be useful to keep calibration values or setup data.
		*/
		class FileConfig : public Config {
		public:
			/**
			* Creates a configuration
			* @param path Name of the configuration file.
			*/
			FileConfig(const char *path = nullptr);
			/**
			* Destructor, do not call manually.
			*/
			virtual ~FileConfig();
			
			/**
			* Saves a configuration, must be overwritten by a derived clas
			* @param path Name of the configuration file.
			* @return true if successful
			*/
			virtual bool save(const char *path = nullptr);
			/**
			* Loads a configuration, must be overwritten by a derived clas
			* @param path Name of the configuration file.
			* @return true if successful
			*/
			virtual bool load(const char *path = nullptr);
			
			static constexpr int buffer_size = 256;
		};
	};
};

#endif // ORG_EEROS_CONFIG_FILECONFIG_HPP_
