#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/ros/RosPublisherDoubleArray.hpp>
#include <eeros/control/ros/RosPublisherDouble.hpp>
#include <eeros/control/ros/RosPublisherSafetyLevel.hpp>
#include <eeros/control/ros/RosSubscriberDoubleArray.hpp>
#include <eeros/control/ros/RosSubscriberDouble.hpp>
#include <ros/ros.h>
#include <signal.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::hal;

class MyControlSystem {
public:
	MyControlSystem(double dt):
		c1({2.4, 0, 4.443, 23.6, -11.2, 1.3, 0.003}),
		c2(0.5),
		vectorOut("/test/vector"),
		doubleOut("/test/val"),
		slOut("/test/safetyLevel"),
		vectorIn("/rosNodeTalker/vector"),
		doubleIn("/rosNodeTalker/val"),
		timedomain("Main time domain", dt, true) 
	{
 		vectorOut.getIn().connect(c1.getOut());
		doubleOut.getIn().connect(c2.getOut());
		timedomain.addBlock(c1);
		timedomain.addBlock(c2);
		timedomain.addBlock(vectorOut);
		timedomain.addBlock(doubleOut);
		timedomain.addBlock(slOut);
		timedomain.addBlock(vectorIn);
		timedomain.addBlock(doubleIn);
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	typedef eeros::math::Matrix<7, 1, double> Vector7;
	Constant<Vector7> c1;
	Constant<> c2;
	RosPublisherDoubleArray<Vector7> vectorOut;	
	RosPublisherDouble doubleOut;	
	RosPublisherSafetyLevel slOut;	
	RosSubscriberDoubleArray<Vector2> vectorIn;	
	RosSubscriberDouble doubleIn;
	TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties(MyControlSystem& cs) : slOne("one"), slTwo("two"), se("change") {	
		addLevel(slOne);
		addLevel(slTwo);
		slOne.addEvent(se, slTwo, kPrivateEvent);
		slTwo.addEvent(se, slOne, kPublicEvent);
		setEntryLevel(slOne);
		
		slOne.setLevelAction([&](SafetyContext* privateContext) {
			cs.c1.setValue(cs.c1.getValue() + diff);
			cs.c2.setValue(cs.c2.getValue() - diff);
			if ((slOne.getNofActivations() % 10) == 0) {
				log.info() << cs.doubleIn.getOut().getSignal();
				log.info() << cs.vectorIn.getOut().getSignal();
			}
			if ((slOne.getNofActivations() % 50) == 0) {
				privateContext->triggerEvent(se);
			}
		});

		slTwo.setLevelAction([&](SafetyContext* privateContext) {
			cs.c1.setValue(cs.c1.getValue() - diff);
			cs.c2.setValue(cs.c2.getValue() + diff);
			if ((slTwo.getNofActivations() % 10) == 0) {
				log.info() << cs.doubleIn.getOut().getSignal();
				log.info() << cs.vectorIn.getOut().getSignal();
			}
			if ((slTwo.getNofActivations() % 50) == 0) {
				privateContext->triggerEvent(se);
			}
		});

	}

	SafetyLevel slOne;
	SafetyLevel slTwo;
	SafetyEvent se;
	Logger log;
	double diff = 0.1;
};

void signalHandler(int signum) {
	Executor::stop();
}

int main(int argc, char **argv) {	
	double dt = 0.1;

	StreamLogWriter w(std::cout);
	Logger::setDefaultWriter(&w);
	Logger log;
	w.show();
 
	log.info() << "ROS Test1 started";

	rosTools::initNode("eerosNode");
	log.trace() << "ROS node initialized.";
		
	MyControlSystem controlSystem(dt);
	MySafetyProperties safetyProperties(controlSystem);
	SafetySystem safetySystem(safetyProperties, dt);
	controlSystem.slOut.setSafetySystem(safetySystem);
	
	signal(SIGINT, signalHandler);	
	auto &executor = Executor::instance();
	executor.setMainTask(safetySystem);
	executor.run();
	
	log.info() << "ROS Test1 end";
	return 0;
}
