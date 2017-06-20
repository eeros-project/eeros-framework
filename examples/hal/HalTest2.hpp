#ifndef ORG_EEROS_HALTEST2_HPP_
#define ORG_EEROS_HALTEST2_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <unistd.h>

using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::sequencer;
using namespace eeros::hal;

const double dt = 0.001;


class MyControlSystem {
public:
	MyControlSystem(double ts): c1(false), c2(1), io1("io1"), ioOut("ioOut"), ioIn("ioIn"), dac1("dac1"),
					encMot1("encMot1"), timedomain("Main time domain", ts, true) {
		ioOut.getIn().connect(ioIn.getOut());
		io1.getIn().connect(c1.getOut());
		dac1.getIn().connect(c2.getOut());
		timedomain.addBlock(c1);
		timedomain.addBlock(c2);
		timedomain.addBlock(io1);
		timedomain.addBlock(ioIn);
		timedomain.addBlock(ioOut);
		timedomain.addBlock(encMot1);
		timedomain.addBlock(dac1);
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	Constant<bool> c1;
	Constant<> c2;
	PeripheralOutput<double> dac1;		// analog output
	PeripheralOutput<bool> io1;		// digital output 
	PeripheralOutput<bool> ioOut;		// digital output
	PeripheralInput<bool> ioIn;		// digital input
	PeripheralInput<double> encMot1;	// encoder input
	TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties() : slSingle("single level") {
		addLevel(slSingle);
		setEntryLevel(slSingle);
	}
	virtual ~MySafetyProperties() { }
	SafetyLevel slSingle;
};

class MyMainSequence : public Sequence<> {
public:
	MyMainSequence(Sequencer* sequencer, MyControlSystem& controlSys) : Sequence<void>("main", sequencer), controlSys(controlSys) { }
	
	virtual bool checkPreCondition() {return true;}
	virtual void run() {
		log.trace() << "[ Main Sequence Started ]";
		
		// set PWM frequency here for example or in main of application
		HAL& hal = HAL::instance();	
	// 	hal.callOutputFeature(&pwm1, "setPwmFrequency", 100.0);
		
		log.info() << "Starting...";
		for(int i = 0; (i < 1000000) && (!isTerminating()); i++){
			if(i%5 == 0){
				log.info() << "enc: " << controlSys.encMot1.getOut().getSignal().getValue();
			}
			if(i%2 == 0){
				controlSys.c2.setValue(-5);
				controlSys.c1.setValue(true);
			}
			else{
				controlSys.c2.setValue(5);
				controlSys.c1.setValue(false);
			}
			usleep(100000);
		}
	}

	virtual void exit() {log.info() << "[ Exit Main Sequence ]";}
	
private:
	inline bool isTerminating() {return sequencer->getState() == state::terminating;}
	MyControlSystem& controlSys;
};

#endif // ORG_EEROS_HALTEST2_HPP_
