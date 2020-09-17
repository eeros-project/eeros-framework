#ifndef ORG_EEROS_ROS_EXAMPLE_HPP_
#define ORG_EEROS_ROS_EXAMPLE_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/control/ros/RosPublisherDouble.hpp>
#include <eeros/control/ros/RosSubscriberLaserScan.hpp>
#include <eeros/control/ros/RosPublisherLaserScan.hpp>
#include <eeros/core/Executor.hpp>
#include <unistd.h>
#include <eeros/logger/Logger.hpp>

using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::hal;
using namespace eeros::logger;

class MyControlSystem {
public:
	MyControlSystem(double ts):
		analogIn0("scanTimeIn0"),		// argument has to match signalId of json
		digitalIn0("batteryPresent0"),
		analogOut0("scanTimeEchoOut0"),
		digitalOut0("batteryPresentEchoOut0"),
		
		laserScanIn ("/rosNodeTalker/TestTopic4", 100, false),
		laserScanOut("/rosExample/TestTopic23", "laser", 100),

		debugOut0("debugNode/debugOut0"),
		dt(ts),
		
		timedomain("Main time domain", dt, true) 
		{
		analogOut0.getIn().connect(analogIn0.getOut());
		digitalOut0.getIn().connect(digitalIn0.getOut());
		
		// Connect ROS blocks
		laserScanOut.getRangesInput().connect(laserScanIn.getRangesOutput());
		laserScanOut.getIntensitiessInput().connect(laserScanIn.getIntensitiesOutput());
		
		// Connect ROS block to publish simple eeros signal
		debugOut0.getIn().connect(analogIn0.getOut());
		
		// Run blocks
		timedomain.addBlock(analogIn0);
		timedomain.addBlock(digitalIn0);
		timedomain.addBlock(laserScanIn);
		timedomain.addBlock(analogOut0);
		timedomain.addBlock(digitalOut0);
		timedomain.addBlock(laserScanOut);
		timedomain.addBlock(debugOut0);
		
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }

	// HAL inputs/outputs
	PeripheralInput<double>		analogIn0;
	PeripheralInput<bool>		digitalIn0;
	PeripheralOutput<double>	analogOut0;
	PeripheralOutput<bool>		digitalOut0;
	
	// ROS blocks
	typedef eeros::math::Matrix< 5, 1, double >		TRangesOutput;
	typedef eeros::math::Matrix< 5, 1, double >		TIntensitiesOutput;
	RosSubscriberLaserScan<TRangesOutput, TIntensitiesOutput>	laserScanIn;
	typedef eeros::math::Matrix< 5, 1, double >		TRangesInput;
	typedef eeros::math::Matrix< 5, 1, double >		TIntensitiesInput;
	RosPublisherLaserScan<TRangesInput, TIntensitiesInput>		laserScanOut;
	
	// Simple ROS block for easy debugging
	RosPublisherDouble		debugOut0;
	
	double dt;
	bool realtime;
	eeros::control::TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties(MyControlSystem& cs) : slOff("off"), cs(cs) {	
		addLevel(slOff);
		setEntryLevel(slOff);
		slOff.setLevelAction([&](SafetyContext* privateContext) {
			if ((slOff.getNofActivations() % 5) == 0) {
				log.info() << cs.analogIn0.getOut().getSignal();
				log.info() << cs.digitalIn0.getOut().getSignal();
			}
		});
	}
	
	SafetyLevel slOff;
	MyControlSystem& cs;
	Logger log;
};


#endif // ORG_EEROS_ROS_EXAMPLE_HPP_