#include <iostream>
#include <array>

#include <eeros/core/SimpleConfig.hpp>


namespace {

	class MyConfig : public eeros::SimpleConfig {
	public:
		MyConfig(const char *name) :
			SimpleConfig(name),
			// default values
			value1(1),
			value2(2.2),
			value3(-3.3),
			value4{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
			value5{ 0.1, 0.2, 0.3 }
		{
			add("value1", value1);
			add("value2", value2);
			add("value3", value3);
			add("value4", value4);
			add("value5", value5);
		}

		int value1;
		double value2, value3;
		std::array<int, 10> value4;
		std::array<double, 3> value5;
	};

}

int main(int argc, char **argv) {
    std::cout << "Config demo started!" << std::endl;

	MyConfig configFile("config.txt");	// choose an appropriate path
	configFile.load();

	std::cout << "Config read from file: value1 = " << configFile.value1 << std::endl;
	std::cout << "Config read from file: value2 = " << configFile.value2 << std::endl;
	std::cout << "Config read from file: value3 = " << configFile.value3 << std::endl;

	std::cout << "Config read from file: value4 = ";
	for (auto &x: configFile.value4) std::cout << x << ", ";
	std::cout << std::endl;

	std::cout << "Config read from file: value5 = ";
	for (auto &x: configFile.value5) std::cout << x << ", ";
	std::cout << std::endl;

	configFile.value1 += 100;
	configFile.value2 += 3.5;
	configFile.value4[3] += 7;
	configFile.value5[1] += 0.05;

	configFile.save();

    return 0;
}
