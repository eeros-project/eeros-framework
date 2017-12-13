#include <eeros/config/SimpleConfig.hpp>
#include <eeros/core/Fault.hpp>

#include <fstream>

using namespace eeros;

namespace {
	bool is_whitespace(char c) {
		return (c <= ' ');
	}
	
	bool is_alpha(char c) {
		return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
	}
};


SimpleConfig::SimpleConfig(const char *path) : Config(path) { }

SimpleConfig::~SimpleConfig() { }

bool SimpleConfig::save(const char *path) {
	if (path == nullptr) path = this->path;
	if (path == nullptr) throw Fault("path is null");
	char buffer[buffer_size + 1];
	std::ofstream file(path);
	if (file.fail()) return false;
	for (auto p: properties ) {
		int n = strlen(p.first);
		strncpy(buffer, p.first, n);
		strcpy(&buffer[n], " = ");
		n += 3;
		int m = p.second.set(p.first, &buffer[n], buffer_size);
		if (m >= 0) n += m;
		if (n > (buffer_size - 1)) n = (buffer_size - 1);
		buffer[n++] = '\n';
		buffer[n] = 0;
		file << buffer;
	}
	return true;
}

bool SimpleConfig::load(const char *path) {
	if (path == nullptr) path = this->path;
	if (path == nullptr) throw Fault("path is null");
	char buffer[buffer_size + 1];
	std::ifstream file(path);
	if (file.fail()) return false;
	while (file.good()) {
		file.getline(buffer, buffer_size);
		int n = strlen(buffer);
		buffer[n] = 0;
		int i = 0;
		
		// skip whitespace
		for (; i < n; i++) {
			if (!is_whitespace(buffer[i]))
				break;
		}

		// scan name
		int name_start = i;
		for (; i < n; i++) {
			if (!is_alpha(buffer[i]))
				break;
		}
		int name_end = i;
		
		// skip whitespace
		for (; i < n; i++) {
			if (!is_whitespace(buffer[i]))
				break;
		}
		
		if (buffer[i++] != '=')
			continue; // illegal format -> skip line

		buffer[name_end] = 0;
		
		auto p = properties.find(&buffer[name_start]);
		if (p == properties.end()) continue; // unknown property
		
		p->second.get(p->first, &buffer[i], (n - i));
	}
	return true;
}
