#include <iostream>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Gate.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Block1i.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::logger;

class LoggerBlock : public Block1i<double> {

public:
	LoggerBlock(Logger<LogWriter> l) : log(l) { }
	
	virtual void run() {
		log.info() << in.getSignal().getValue();
	}
	
private:
	Logger<LogWriter> log;
};

template < typename T = double>
class ContainerBlock : public Block {
	
public:
	ContainerBlock() : gain1(2), gain2(4), gain3(8) {
		gain1.getIn().connect(in);
		gain2.getIn().connect(gain1.getOut());
		gain3.getIn().connect(gain2.getOut());
		out.connect(gain3.getOut());
	}
	
	virtual Input<T>& getIn() {
		return dynamic_cast<Input<T>&>(in);
	}
	
	virtual Output<T>& getOut() {
		return dynamic_cast<Output<T>&>(out);
	}
public:
	virtual void run() {
		gain1.run();
		gain2.run();
		gain3.run();
	}
	
private:
	Gate<T> in;
	Gate<T> out;
	
	Gain<T, double> gain1, gain2, gain3;
};

int main() {
	// Create and initialize logger
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Martin Test 1 started...";
	
	// Create the control system
	TimeDomain td("main", 0.1, false);
	Constant<double> c(5);
	ContainerBlock<double> g;
	LoggerBlock l(log);
	
	g.getIn().connect(c.getOut());
	l.getIn().connect(g.getOut());
	
	td.addBlock(&c); td.addBlock(&g); td.addBlock(&l);
	
	td.start();
	sleep(5);
	td.stop();
	td.join();
	
	log.info() << "Example finished...";
}
