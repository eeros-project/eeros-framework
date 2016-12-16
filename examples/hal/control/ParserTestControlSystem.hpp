#ifndef CH_NTB_PARSERTESTCONTROLSYSTEM_HPP_
#define CH_NTB_PARSERTESTCONTROLSYSTEM_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>

class ParserTestControlSystem {
  
public:
	ParserTestControlSystem(double ts);
	virtual ~ParserTestControlSystem();
	
	eeros::control::Constant<bool> setPos;
	eeros::control::PeripheralOutput<double> dac1;
	eeros::control::PeripheralOutput<bool> io1;
	eeros::control::PeripheralOutput<bool> ioOut;
	eeros::control::PeripheralInput<bool> ioIn;
	eeros::control::PeripheralInput<double> encMot1;
	eeros::control::PeripheralOutput<double> pwm1;
	eeros::control::TimeDomain timedomain;
  
private:
// 	eeros::control::PeripheralInput<bool> readySig1;
	// ...
};

#endif // CH_NTB_PARSERTESTCONTROLSYSTEM_HPP_