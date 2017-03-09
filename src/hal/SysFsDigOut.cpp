#include <eeros/hal/SysFsDigOut.hpp>
#include <eeros/core/Fault.hpp>

using namespace eeros::hal;

SysFsDigOut::SysFsDigOut(std::string id, unsigned int gpio, bool inverted) : PeripheralOutput<bool>(id), basePath("/sys/class/gpio/gpio" + std::to_string(gpio) + "/") {
	std::ofstream exportFile;
	std::ofstream directionFile;
	
	// Export GPIO
	exportFile.open("/sys/class/gpio/export");
	if(!exportFile.is_open()) {
		throw Fault("Failed to export GPIO" +  std::to_string(gpio) + "!");
	}
	exportFile << gpio;
	exportFile.close();
	
	// Set GPIO direction to output
	directionFile.open(basePath + "direction");
	if(!directionFile.is_open()) {
		throw Fault("Failed to set direction to output for GPIO" +  std::to_string(gpio) + "!");
	}
	directionFile << "out";
	directionFile.close();
	
	// Open value file
	valueFile.open(basePath + "value");
	if(!valueFile.is_open()) {
		throw Fault("Failed to open value file for GPIO" +  std::to_string(gpio) + "!");
	}
}

SysFsDigOut::~SysFsDigOut(){
	valueFile.close();
}
bool SysFsDigOut::get() {
	int value;
	if(!valueFile.is_open()) return false;
	valueFile >> value;
	if(inverted) value = !value;
	return static_cast<bool>(value);
}

void SysFsDigOut::set(bool value) {
	if(inverted) value = !value;
	if(valueFile.is_open()) {
		valueFile << static_cast<int>(value);
	}
}
