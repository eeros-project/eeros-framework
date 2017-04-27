#ifndef ORG_EEROS_HALTEST1_HPP_
#define ORG_EEROS_HALTEST1_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/hal/HAL.hpp>
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
	MyControlSystem(double ts): digIn0("dIn0"), digIn1("dIn1"), digOut0("dOut0"), digOut1("dOut1"), 
					anIn0("aIn0"), anIn2("aIn2"), anOut0("aOut0"), anOut2("anOut2"),
					c0(true), c1(-2.0), c2(3.0), timedomain("Main time domain", ts, true) {
		digIn0.getOut().getSignal().setName("digital input 0");
		digIn1.getOut().getSignal().setName("digital input 1");
		anIn0.getOut().getSignal().setName("analog input 0");
		anIn2.getOut().getSignal().setName("analog input 2");
		anOut0.getIn().connect(c1.getOut());
		anOut2.getIn().connect(c2.getOut());
		digOut0.getIn().connect(c0.getOut());
		digOut1.getIn().connect(digIn0.getOut());
		timedomain.addBlock(c0);
		timedomain.addBlock(digOut0);
		timedomain.addBlock(digIn0);
		timedomain.addBlock(digOut1);
		timedomain.addBlock(digIn1);
		timedomain.addBlock(c1);
		timedomain.addBlock(c2);
		timedomain.addBlock(anOut0);
		timedomain.addBlock(anIn0);
		timedomain.addBlock(anOut2);
		timedomain.addBlock(anIn2);
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	Constant<bool> c0;
	Constant<> c1, c2;
	PeripheralOutput<bool> digOut0, digOut1;
	PeripheralInput<bool> digIn0, digIn1;
	PeripheralOutput<double> anOut0, anOut2;
	PeripheralInput<double> anIn0;
	PeripheralInput<double> anIn2;
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
		bool toggle = false;
		bool toggleAnalog = false;
		log.trace() << "[ Main Sequence Started ]";
		sleep(4);
		
		log.info() << "Starting...";
		for(int i = 0; (i < 1000000) && (!isTerminating()); i++){	  
			if(i%20 == 0){
				log.trace() << controlSys.digIn0.getOut().getSignal();
				log.trace() << controlSys.digIn1.getOut().getSignal();
				log.trace() << controlSys.anIn0.getOut().getSignal();
				log.trace() << controlSys.anIn2.getOut().getSignal();
			}
			if(i%50 == 0){
				toggle = !toggle;
				log.info() << "toggle digital output of constant block c0";
				controlSys.c0.setValue(toggle);
			}
			if(i%100 == 0){
				toggleAnalog = !toggleAnalog;
				log.info() << "change analog output values of constant blocks c1 and c2";
				if(toggleAnalog){
					controlSys.c1.setValue(6.0);
					controlSys.c2.setValue(-3.2);
				} else {
					controlSys.c1.setValue(-2.0);
					controlSys.c2.setValue(7.6);
				}
			}
			usleep(100000);
		}
	}

	virtual void exit() {log.info() << "[ Exit Main Sequence ]";}
	
private:
	inline bool isTerminating() {return sequencer->getState() == state::terminating;}
	MyControlSystem& controlSys;
};

#endif // ORG_EEROS_HALTEST1_HPP_
