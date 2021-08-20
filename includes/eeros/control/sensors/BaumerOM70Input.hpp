#ifndef ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP
#define ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP

#include <string>
#include <thread>
#include <eeros/control/Blockio.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/BaumerOM70.hpp>


using namespace eeros::math;
using namespace eeros::hal;
using namespace eeros::logger;

namespace eeros {
namespace control {

	class BaumerOM70Input: public eeros::control::Blockio<0,1,double> {
	public:
		/**
		* Constructs an input block to get data from baumer OM70 sensor. Output is a dlaser distance \n
		* Calls BaumerOM70Input(std::string dev, int port, int slave, int priority)
		*
		* @see  BaumerOM70Input(std::string dev, int port, int slave, int priority)
		* @param dev - string with device name
		* @param port - port for sensor data read (modbus interface)
		* @param slave - sensor slave number (modbus interface)
		* @param priority - execution priority or BaumerOM70 thread, to get sensors data
		*/
		BaumerOM70Input(std::string dev, int port, int slave, int priority = 5);
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		BaumerOM70Input(const BaumerOM70Input& s) = delete; 
		
		/**
		* Gets input data from Baumer OM70 Thread and outputs them
		*/
		virtual void run();
        
	protected:        
        eeros::hal::OM70 om70;
        eeros::logger::Logger log;
	};
};
}


#endif /* ORG_EEROS_CONTROL_BAUMEROM70_INPUT_HPP */


