#include <iostream>
#include <eeros/core/SimpleConfig.hpp>

#include <sys/types.h>
#include <sys/stat.h>


#define ERROR  (error++, std::cerr << "line: " << __LINE__ << std::endl)


class TestConfig : public eeros::SimpleConfig {
public:
	TestConfig(const char *name) : SimpleConfig(name) {
		add("value1", value1);
		add("value2", value2);
	}
	
	virtual void loadDefaults() {
		value1 = 17;
		value2 = 584.25;
	}
	
	virtual void zero() {
		value1 = 0;
		value2 = 0.0;
	}
	
	int value1;
	double value2;
};


bool dir_exists(const char *path) {
	struct stat info;

	if (stat(path, &info) != 0)
		return false;

    return (info.st_mode & S_IFDIR);
}

bool file_exists(const char *path) {
	struct stat info;

	if (stat(path, &info) != 0)
		return false;

    return (info.st_mode & S_IFREG);
}

std::string trim(const std::string& s, const char *chars = "/ \t")
{
    auto end = s.find_last_not_of(chars);
	if (end == std::string::npos)
		return s;
	else
		return s.substr(0, end+1);
}


int main(int argc, char *argv[]) {
	int error = 0;
	const char *dir_path = "/tmp/eeros-tests/";
	
	if (argc == 2) {
		dir_path = argv[1];
	}
	
	if (!dir_exists(dir_path)) {
		std::cerr << "parameter is not a directory: " << dir_path << std::endl;
		return -2;
	}
	
	std::string dir = trim(std::string(dir_path));
	std::string defaultConfigFile(dir + "/config-default.conf");
	std::string v1ConfigFile(dir + "/config-v1.conf");
	
	TestConfig config(defaultConfigFile.c_str());
	config.loadDefaults();
	config.save();
	
	config.value1 = 31;
	config.value2 = -9876.2145;
	config.save(v1ConfigFile.c_str());
	
	
	TestConfig configDefault(defaultConfigFile.c_str());
	configDefault.zero();
	configDefault.load();
	
	if (configDefault.value1 != 17) ERROR;
	if (configDefault.value2 != 584.25) ERROR;
	
	
	TestConfig configV1(v1ConfigFile.c_str());
	configV1.zero();
	configV1.load();
	
	if (configV1.value1 != 31) ERROR;
	if (configV1.value2 != -9876.2145) ERROR;
	
	return error;
}
