#include <eeros/hal/LinuxDigOut.hpp>
#include <eeros/core/EEROSException.hpp>
using namespace eeros::hal;
#include <fcntl.h>
#include <unistd.h>
LinuxDigOut::LinuxDigOut(std::string id,uint32_t gpioNr, bool inverted) : PeripheralOutput<bool>(id) {
	int32_t fd;
	char filePath[256];
	char buf[10];
	sprintf(filePath,"/sys/class/gpio/gpio%d/direction",gpioNr);

	//Export GPIO
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if(fd < 0){
		//ERROR
		throw EEROSException("Can't export GPIO: \"" +  std::to_string(gpioNr) + "\"!"); // TODO define error number and send error message to logger
	}
	sprintf(buf, "%d", gpioNr);
	write(fd,buf,sizeof(buf));
	close(fd);
	//Set Direction to Output
	fd = open(filePath, O_WRONLY);
	if(fd < 0 ){
		//ERROR
		throw EEROSException("Can't open direction file GPIO: \"" +  std::to_string(gpioNr) + "\"!"); // TODO define error number and send error message to logger
	}
	write(fd,"out",sizeof("out"));
	close(fd);
	sprintf(filePath,"/sys/class/gpio/gpio%d/value",gpioNr);
	valueFd = open(filePath,O_RDWR);
	if(valueFd < 0 ){
		throw EEROSException("Can't open value file GPIO: \"" +  std::to_string(gpioNr) + "\"!"); // TODO define error number and send error message to logger
	}

}

LinuxDigOut::~LinuxDigOut(){
	close(valueFd);
}
bool LinuxDigOut::get() {
	char value;
	bool ret;
	lseek(valueFd,0,SEEK_SET);
	read(valueFd, &value, 1);
	if(value == '1'){
		ret = true;
	}else{
		ret = false;	
	}
	if(inverted){
		return !ret;
	}else{
		return ret;	
	}
}

void LinuxDigOut::set(bool value) {
	if(inverted) value = !value;
	lseek(valueFd,0,SEEK_SET);
	if(value == true){
		write(valueFd, "1", 1); 
	}else{
		write(valueFd, "0", 1);
	}
}
