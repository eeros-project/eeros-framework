#ifndef ORG_EEROS_HALTEST1_HPP_
#define ORG_EEROS_HALTEST1_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Step.hpp>
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

class StepDigOut : public Step {
public:
	StepDigOut(std::string name, Sequencer& sequencer, BaseSequence* caller, MyControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int operator() (bool val) {state = val; return Step::start();}
	int action() {cs.c0.setValue(state);}
	bool state;
	MyControlSystem& cs;
};

class StepAnalogOut : public Step {
public:
	StepAnalogOut(std::string name, Sequencer& sequencer, BaseSequence* caller, MyControlSystem& cs) : Step(name, sequencer, caller), cs(cs) { }
	int operator() (bool val) {state = val; return Step::start();}
	int action() {
		if (state) {
			cs.c1.setValue(6.0);
			cs.c2.setValue(-3.2);
		} else {
			cs.c1.setValue(-2.0);
			cs.c2.setValue(7.6);
		}
	}
	bool state;
	MyControlSystem& cs;
};

class SeqDigital : public Sequence {
public:
	SeqDigital(std::string name, Sequencer& sequencer, BaseSequence* caller, MyControlSystem& cs) : Sequence(name, sequencer, caller), stepDigOut("step dig out", seq, this, cs) {
		setNonBlocking();
	}
	int action() {
		bool toggle;
		while (true) {
			stepDigOut(toggle);
			sleep(2);
			toggle = !toggle;
		}
	}
private:
	StepDigOut stepDigOut;
};

class SeqAnalog : public Sequence {
public:
	SeqAnalog(std::string name, Sequencer& sequencer, BaseSequence* caller, MyControlSystem& cs) : Sequence(name, sequencer, caller), stepAnalogOut("step analog out", seq, this, cs) {
		setNonBlocking();
	}
	int action() {
		bool toggle;
		while (true) {
			stepAnalogOut(toggle);
			sleep(5);
			toggle = !toggle;
		}
	}
private:
	StepAnalogOut stepAnalogOut;
};

class MyMainSequence : public Sequence {
public:
	MyMainSequence(Sequencer& seq, MyControlSystem& cs) : Sequence("main", seq), cs(cs), seqDigital("seq dig out", seq, this, cs), seqAnalog("seq analog out", seq, this, cs) { 
		setNonBlocking();
	}
	
	int action() {
		seqDigital();
		seqAnalog();
		while (true) {
			sleep(1);
			log.info() << cs.digIn0.getOut().getSignal();
			log.info() << cs.digIn1.getOut().getSignal();
			log.info() << cs.anIn0.getOut().getSignal();
			log.info() << cs.anIn2.getOut().getSignal();
		}
		seqDigital.join();
		seqAnalog.join();
	}
private:
	MyControlSystem& cs;
	SeqDigital seqDigital;
	SeqAnalog seqAnalog;
};

#endif // ORG_EEROS_HALTEST1_HPP_
