#include <iostream>
#include <array>
#include <eeros/config/FileConfig.hpp>

namespace {

class TestConfig : public eeros::config::FileConfig {
 public:
  TestConfig(const char *name) 
      : FileConfig(name),
        value1(1),      // default values
        value2(2.2),
        value3(-3.3),
        value4{0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
        value5{0.1, 0.2, 0.3},
        value6("hello") {
    add("value1", value1);
    add("value2", value2);
    add("value3", value3);
    add("value4", value4);
    add("value5", value5);
    add("value6", value6);
  }

  int value1;
  double value2, value3;
  std::array<int, 10> value4;
  std::array<double, 3> value5;
  std::string value6;
};
}

int main(int argc, char **argv) {
  std::cout << "Config demo started" << std::endl;

  TestConfig configFile("config.txt");	// choose an appropriate path
  std::cout << "Config default: value1 = " << configFile.value1 << std::endl;
  std::cout << "Config default: value2 = " << configFile.value2 << std::endl;
  std::cout << "Config default: value3 = " << configFile.value3 << std::endl;
  std::cout << "Config default: value4 = ";
  for (auto &x: configFile.value4) std::cout << x << ", ";
  std::cout << std::endl;
  std::cout << "Config default: value5 = ";
  for (auto &x: configFile.value5) std::cout << x << ", ";
  std::cout << std::endl;
  std::cout << "Config default: value6 = " << configFile.value6 << std::endl;

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
  std::cout << "Config read from file: value6 = " << configFile.value6 << std::endl;

  configFile.value1 += 100;
  configFile.value2 = 3.5;
  configFile.value4[3] += 7;
  configFile.value5[1] += 0.05;
  configFile.value5[2] = 2.345;

  configFile.save();

  return 0;
}
