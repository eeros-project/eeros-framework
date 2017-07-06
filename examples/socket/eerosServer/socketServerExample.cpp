#include <eeros/logger/Logger.hpp>
#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Constant.hpp>
#include <eeros/control/SocketData.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>

using namespace eeros::control;
using namespace eeros::sockets;
using namespace eeros::logger;

namespace testapptcpip {
	
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
			
			eeros::Executor::instance().add(timedomain);
			
		}
		
		// Define blocks
		Constant<eeros::math::Vector4> c1;
		eeros::control::SocketData<4, double, eeros::math::Vector4> socketA;
		Logger log;
		
	protected:
		double dt;
		bool realtime;
		eeros::control::TimeDomain timedomain;
	}; // END class
	
}; // END namespace

#include <eeros/safety/SafetyProperties.hpp>
 
namespace testapptcpip {
 
    class TestAppSafetyProperties : public eeros::safety::SafetyProperties {
 
    public: 
	TestAppSafetyProperties() : off("System off") {
		
		addLevel(off);
		setEntryLevel(off);
	}
	
	~TestAppSafetyProperties(){};
 
	eeros::safety::SafetyLevel off;
	
    private:
 
    }; // end class
};     // end namespace

#include <iostream>
#include <signal.h>

#include <eeros/core/Executor.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/sockets/SocketServer.hpp>
#include <eeros/task/Periodic.hpp>

using namespace eeros;
using namespace eeros::logger;
using namespace testapptcpip;
	
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
	testapptcpip::TestAppCS controlSystem (dt);
	
	// Safety System
	TestAppSafetyProperties safetyProperties;
	eeros::safety::SafetySystem safetySystem(safetyProperties, dt);

	eeros::task::Lambda l1;
	eeros::task::Periodic periodic("per1", 1.0, l1);
	periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
		log.info() << controlSystem.socketA.getOut().getSignal();
	});
		
	// Create and run executor
	auto& executor = eeros::Executor::instance();
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

