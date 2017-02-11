#ifndef CH_NTB_SIMCONTROLSYSTEM_HPP_
#define CH_NTB_SIMCONTROLSYSTEM_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>

class SimControlSystem {
  
public:
	SimControlSystem(double ts);
	virtual ~SimControlSystem();
	
	eeros::control::PeripheralOutput<bool> simOut_out0;
	eeros::control::PeripheralInput<bool> simOut_in0;
	
// 	eeros::control::PeripheralOutput<double> aOut0;
	eeros::control::PeripheralInput<double> aInTest0;
	eeros::control::PeripheralInput<double> aIn2;
	eeros::control::PeripheralOutput<double> aOutTest2;
	eeros::control::TimeDomain timedomain;
  
private:
// 	eeros::control::PeripheralInput<bool> readySig1;
	// ...
};

#endif // CH_NTB_SIMCONTROLSYSTEM_HPP_