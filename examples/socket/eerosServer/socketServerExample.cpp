#include <eeros/logger/Logger.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Constant.hpp>
#include <eeros/control/SocketData.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>

using namespace eeros::control;
using namespace eeros::sockets;
using namespace eeros::logger;
using namespace eeros::math;
using namespace eeros::task;
using namespace eeros;

class TestAppCS {
public:
	TestAppCS(double dt) : 
		dt(dt),
		log('C'),
		socketA(9876, 0.01),
		c1({1.5, 2.2, 3.3, 4.6}),
		timedomain("Main time domain", dt, true){
		
		socketA.getOut().getSignal().setName("socketRead");
		  
		// Connect Blocks
		socketA.getIn().connect(c1.getOut());
		
		// Run blocks
		timedomain.addBlock(c1);
		timedomain.addBlock(socketA);
		
		Executor::instance().add(timedomain);
		
	}
		
	// Define blocks
	Constant<Vector4> c1;
	SocketData<Vector4, double, Matrix<1000,1,double>, double> socketA;
	Logger log;
		
protected:
	double dt;
	bool realtime;
	TimeDomain timedomain;
}; // END class
	

#include <eeros/safety/SafetyProperties.hpp>
 
class TestAppSafetyProperties : public eeros::safety::SafetyProperties {
public: 
	TestAppSafetyProperties() : off("System off") {
		addLevel(off);
		setEntryLevel(off);
	}
	
	~TestAppSafetyProperties(){};
 
	eeros::safety::SafetyLevel off;
	
}; // end class

#include <iostream>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sockets/SocketServer.hpp>
#include <eeros/task/Periodic.hpp>
	
bool threadFinishd = false;

volatile bool running = true;
void signalHandler(int signum) {
	running = false;
}


int main(int argc, char **argv) {
	signal(SIGINT, signalHandler);
	
	double dt = 0.01;
	
	StreamLogWriter w(std::cout);
	w.show(LogLevel::TRACE);
	Logger::setDefaultWriter(&w);
	Logger log;
 
	log.info() << "EEROS started";
	
	// Control System
	TestAppCS controlSystem (dt);
	
	// Safety System
	TestAppSafetyProperties safetyProperties;
	SafetySystem safetySystem(safetyProperties, dt);

	Lambda l1;
	Periodic periodic("per1", 1.0, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		log.info() << controlSystem.socketA.getOut().getSignal();
	});
		
	// Create and run executor
	auto& executor = Executor::instance();
	executor.setMainTask(safetySystem);
	executor.add(periodic);
	
// 	// Sequencer
// 	eeros::sequencer::Sequencer S;
// 	MainSequence mainSequence(S, &controlSystem, "mainSequence");
// 	S.addMainSequence(&mainSequence);
	
	log.info() << "executor.run()";
	executor.run();

	threadFinishd = true;
	
	return 0;
}

