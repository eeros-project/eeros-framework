#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <eeros/core/SimpleConfig.hpp>
 
using namespace eeros;
 
class MyConfig : public SimpleConfig {
public:
                MyConfig(const char *name) : SimpleConfig(name) {
                               add("value1", value1);
                               add("value2", value2);
                }
               
                int value1;
                double value2;
};
 
 
int main(int argc, char **argv) {
    std::cout << "Config demo started!" << std::endl;
                std::string fileName("config.txt");          // choose an appropriate path
                MyConfig configFile(fileName.c_str());
                configFile.load();
                std::cout << "Config read from file: value1 = " << configFile.value1 << std::endl;
                std::cout << "Config read from file: value2 = " << configFile.value2 << std::endl;
                configFile.value1 += 100;
                configFile.value2 += 3.5;
                configFile.save();
 
    return 0;
}