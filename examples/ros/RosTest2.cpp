#include <signal.h>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/ROS/RosPublisherDoubleArray.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::hal;

class MyControlSystem {
public:
	MyControlSystem(double dt, ros::NodeHandle& rosNodeHandler):
		rosNodeHandler(rosNodeHandler),
		c1({2.4, 0, 4.443, 23.6, -11.2, 1.3, 0.003}),
		vectorOut(rosNodeHandler, "/rosTest2/Vector", 100),
		timedomain("Main time domain", dt, true) 
	{
 		vectorOut.getIn().connect(c1.getOut());	
		timedomain.addBlock(c1);
		timedomain.addBlock(vectorOut);
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	typedef eeros::math::Matrix<7, 1, double> Vector7;
	Constant<Vector7> c1;
	RosPublisherDoubleArray<Vector7> vectorOut;	
	ros::NodeHandle& rosNodeHandler;
	TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties(MyControlSystem& cs) : slOff("off") {	
		addLevel(slOff);
		setEntryLevel(slOff);
		
		slOff.setLevelAction([&](SafetyContext* privateContext) {
			if(slOff.getNofActivations() > 3) {
				cs.c1.setValue(cs.c1.getOut().getSignal().getValue() + 0.1);
			}
		});
	}

	SafetyLevel slOff;
};

void signalHandler(int signum) {
	Executor::stop();
}

int main(int argc, char **argv) {	
	double dt = 0.2;

	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
 
	log.info() << "ROS Test2 started";

	char* dummy_args[] = {NULL};
	int dummy_argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
	ros::init(dummy_argc, dummy_args, "rosExample");
	ros::NodeHandle rosNodeHandler;
	log.trace() << "ROS node initialized.";
	
	eeros::System::useRosTime();	// "ros::Time::now()" is used to get system time
	
	MyControlSystem controlSystem(dt, rosNodeHandler);
	MySafetyProperties safetyProperties(controlSystem);
	eeros::safety::SafetySystem safetySystem(safetyProperties, dt);
	
	signal(SIGINT, signalHandler);	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
	executor.run();
	
	log.info() << "ROS Test2 end";
	return 0;
}
