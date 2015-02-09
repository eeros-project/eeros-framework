#include <eeros/core/Config.hpp>
#include <eeros/core/EEROSException.hpp>

#include <string>

using namespace eeros;


Config::Config(const char *path) : path(path) { }

Config::~Config() { }

void Config::loadDefaults() { }

void Config::add(const char *name, int &value) {
	auto k = properties.find(name);
	if (k != properties.end()) {
		throw eeros::EEROSException(std::string("Property '") + name + "' already added.");
	}
	properties[name] = ConfigPropertyAccessor{
		[&value] (const char *name, char *buffer, int size) -> int {
			const char *v = std::to_string(value).c_str();
			int n = strlen(v);
			if (n > size) n = size;
			strncpy(buffer, v, n);
			return n;
		},
		[&value] (const char *name, const char *buffer, int size) -> int {
			value = std::stoi(std::string(buffer));
			return 0;
		}
	};
}

void Config::add(const char *name, double &value) {
	auto k = properties.find(name);
	if (k != properties.end()) {
		throw eeros::EEROSException(std::string("Property '") + name + "' already added.");
	}
	properties[name] = ConfigPropertyAccessor{
		[&value] (const char *name, char *buffer, int size) -> int {
			const char *v = std::to_string(value).c_str();
			int n = strlen(v);
			if (n > size) n = size;
			strncpy(buffer, v, n);
			return n;
		},
		[&value] (const char *name, const char *buffer, int size) -> int {
			value = std::stod(std::string(buffer));
			return 0;
		}
	};
}
