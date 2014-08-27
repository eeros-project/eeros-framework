#include <eeros/hal/SysFsDigIn.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::hal;

SysFsDigIn::SysFsDigIn(std::string id, unsigned int gpio, bool inverted) : PeripheralInput<bool>(id), basePath("/sys/class/gpio/gpio" + std::to_string(gpio) + "/") {
	std::ofstream exportFile;
	std::ofstream directionFile;
	
	// Export GPIO
	exportFile.open("/sys/class/gpio/export");
	if(!exportFile.is_open()) {
		throw EEROSException("Failed to export GPIO" +  std::to_string(gpio) + "!");
	}
	exportFile << gpio;
	exportFile.close();
	
	// Set GPIO direction to output
	directionFile.open(basePath + "direction");
	if(!directionFile.is_open()) {
		throw EEROSException("Failed to set direction to output for GPIO" +  std::to_string(gpio) + "!");
	}
	directionFile << "in";
	directionFile.close();
	
	// Open value file
	valueFile.open(basePath + "value");
	if(!valueFile.is_open()) {
		throw EEROSException("Failed to open value file for GPIO" +  std::to_string(gpio) + "!");
	}
}

SysFsDigIn::~SysFsDigIn(){
	valueFile.close();
}
bool SysFsDigIn::get() {
	int value;
	if(!valueFile.is_open()) return false;
	valueFile >> value;
	if(inverted) value = !value;
	return static_cast<bool>(value);
}
