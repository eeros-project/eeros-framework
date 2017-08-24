#ifndef ORG_EEROS_ROS_EXAMPLE_HPP_
#define ORG_EEROS_ROS_EXAMPLE_HPP_

#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/OutputAction.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/control/ROS/RosBlockPublisherDouble.hpp>
#include <eeros/control/ROS/RosBlockSubscriber_SensorMsgs_LaserScan.hpp>
#include <eeros/control/ROS/RosBlockPublisher_SensorMsgs_LaserScan.hpp>
#include <eeros/core/Executor.hpp>
#include <unistd.h>

using namespace eeros::safety;
using namespace eeros::control;
using namespace eeros::hal;


// thsi block prints value of signal
template < typename T = double >
class Print : public eeros::control::Block1i<T> {
	public:
		Print(int modulo=1) : modulo(modulo), counter(0) { }
		virtual void run() {
			if ( (counter % modulo) == 0 ) {
				std::cout << this->in.getSignal().getValue() << std::endl;
			}
			counter++;
		}
		int modulo;
		uint64_t counter;
};


class MyControlSystem {
public:
	MyControlSystem(double ts, ros::NodeHandle& rosNodeHandler):
		dt(ts),
		rosNodeHandler(rosNodeHandler),
		
		printDouble0(1),
		printBool0(1),
		
		analogIn0("scanTimeIn0"),		// argument has to match signalId of json
		digitalIn0("batteryPresent0"),
		analogOut0("scanTimeEchoOut0"),
		digitalOut0("batteryPresentEchoOut0"),
		
		laserScanIn (rosNodeHandler, "/rosNodeTalker/TestTopic3", 100, false),
		laserScanOut(rosNodeHandler, "/rosExample/TestTopic23", 100),

		debugOut0(rosNodeHandler, "debugNode/debugOut0"),
		
		timedomain("Main time domain", dt, true) 
		{
		
		// Connect HAL blocks
		printDouble0.getIn().connect(analogIn0.getOut());
		printBool0.getIn().connect(digitalIn0.getOut());
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
		timedomain.addBlock(printBool0);
		timedomain.addBlock(printDouble0);
		timedomain.addBlock(analogOut0);
		timedomain.addBlock(digitalOut0);
		timedomain.addBlock(laserScanOut);
		timedomain.addBlock(debugOut0);
		
		eeros::Executor::instance().add(timedomain);
	}
	virtual ~MyControlSystem() { }
	
	// Console output
	Print<double> printDouble0;
	Print<bool> printBool0;
	
	// HAL inputs/outputs
	PeripheralInput<double>		analogIn0;
	PeripheralInput<bool>		digitalIn0;
	PeripheralOutput<double>	analogOut0;
	PeripheralOutput<bool>		digitalOut0;
	
	// ROS blocks
	typedef eeros::math::Matrix< 5, 1, double >		TRangesOutput;
	typedef eeros::math::Matrix< 5, 1, double >		TIntensitiesOutput;
	RosBlockSubscriber_SensorMsgs_LaserScan<TRangesOutput, TIntensitiesOutput>	laserScanIn;
	typedef eeros::math::Matrix< 5, 1, double >		TRangesInput;
	typedef eeros::math::Matrix< 5, 1, double >		TIntensitiesInput;
	RosBlockPublisher_SensorMsgs_LaserScan<TRangesInput, TIntensitiesInput>		laserScanOut;
	
	// Simple ROS block for easy debugging
	RosBlockPublisherDouble		debugOut0;
	
	double dt;
	ros::NodeHandle& rosNodeHandler;
	bool realtime;
	eeros::control::TimeDomain timedomain;
};

class MySafetyProperties : public SafetyProperties {
public:
	MySafetyProperties() : slOff("off") {	
		addLevel(slOff);
		setEntryLevel(slOff);
	}
	
	SafetyLevel slOff;
};


#endif // ORG_EEROS_ROS_EXAMPLE_HPP_