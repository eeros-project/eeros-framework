#include <eeros/core/Config.hpp>
#include <eeros/core/EEROSException.hpp>

#include <string>
#include <sstream>

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

void Config::add(const char *name, std::size_t length, int *start, int *end, int default_value) {
	if (start + length != end) {
		throw eeros::EEROSException(std::string("Property '") + name + "': array length inconsistent");
	}
	auto k = properties.find(name);
	if (k != properties.end()) {
		throw eeros::EEROSException(std::string("Property '") + name + "' already added.");
	}
	properties[name] = ConfigPropertyAccessor{
		[length, start] (const char *name, char *buffer, int size) -> int {
			std::stringstream ss;
			for (int i = 0; i < length; i++) {
				if (i != 0) ss << ", ";
				ss << start[i];
			}
			auto property_value_string = ss.str();
			int n = property_value_string.length();
			if (n + 1 >= size) throw eeros::EEROSException(std::string("Property '") + name + "': buffer too small");
			const char *property_value = property_value_string.c_str();
			strncpy(buffer, property_value, n);
			buffer[n] = 0;
			return n;
		},
		[length, start, default_value] (const char *name, const char *buffer, int size) -> int {
			constexpr std::size_t value_buffer_size = (64 - 1);
			char value_buffer[value_buffer_size + 1] = { };

			int current_value = 0;
			int i = 0;
			int j = 0;
			while (true) {
				if (buffer[i] == ',' || i >= size) {
					int n = (i - j);
					if (n >= value_buffer_size) throw eeros::EEROSException(std::string("Property '") + name + "': buffer too small");
					if (n <= 0) break;
					strncpy(value_buffer,&buffer[j], n);
					value_buffer[n] = 0;
					int v = std::atoi(value_buffer);
					start[current_value++] = v;
					j = (i + 1);
				}
				i++;
			}
			while (current_value < length) {
				start[current_value++] = default_value;
			}

			return 0;
		}
	};
}

void Config::add(const char *name, std::size_t length, double *start, double *end, double default_value) {
	if (start + length != end) {
		throw eeros::EEROSException(std::string("Property '") + name + "': array length inconsistent");
	}
	auto k = properties.find(name);
	if (k != properties.end()) {
		throw eeros::EEROSException(std::string("Property '") + name + "' already added.");
	}
	properties[name] = ConfigPropertyAccessor{
		[length, start] (const char *name, char *buffer, int size) -> int {
			std::stringstream ss;
			for (int i = 0; i < length; i++) {
				if (i != 0) ss << ", ";
				ss << start[i];
			}
			auto property_value_string = ss.str();
			int n = property_value_string.length();
			if (n >= size) throw eeros::EEROSException(std::string("Property '") + name + "': buffer too small");
			const char *property_value = property_value_string.c_str();
			strncpy(buffer, property_value, n);
			return n;
		},
		[length, start, default_value] (const char *name, const char *buffer, int size) -> int {
		constexpr std::size_t value_buffer_size = (64 - 1);
		char value_buffer[value_buffer_size + 1] = { };

		int current_value = 0;
		int i = 0;
		int j = 0;
		while (true) {
			if (buffer[i] == ',' || i >= size) {
				int n = (i - j);
				if (n >= value_buffer_size) throw eeros::EEROSException(std::string("Property '") + name + "': buffer too small");
				if (n <= 0) break;
				strncpy(value_buffer,&buffer[j], n);
				value_buffer[n] = 0;
				double v = std::atof(value_buffer);
				start[current_value++] = v;
				j = (i + 1);
			}
			i++;
		}
		while (current_value < length) {
			start[current_value++] = default_value;
		}

		return 0;
		}
	};
}
