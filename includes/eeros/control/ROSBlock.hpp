#ifndef ORG_EEROS_CONTROL_ROSBLOCK_HPP_		// USER DEFINED guard name
#define ORG_EEROS_CONTROL_ROSBLOCK_HPP_

#include <ros/ros.h>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>

// USER DEFINED. Include used message type
//#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/LaserScan.h>

namespace userNamespace {													// USER DEFINED namespace
	class ROSBlockTopic2 : public eeros::control::Block {			// USER DEFINED class name

		// USER DEFINED: Adapt the type definitions for your application.
		//               Multiple outputs are possible.
		//               It is not easily possible, to subscribe to multiple ROS topics with one block
		//               Use "$ rosmsg show sensor_msgs/Joy" to see, which data a certain message type contains
		typedef sensor_msgs::Joy::Type						typeROSMessage;		// Depends on ROS topic.
		typedef eeros::math::Matrix< 3 , 1 , double >		typeAxesOutput;
		typedef eeros::math::Matrix< 5 , 1 , int >			typeButtonsOutput;
		// END OF USER DEFINED


	public:
		ROSBlockTopic2(ros::NodeHandle& rosNodeHandler, const std::string& topic, uint32_t queueSize=1000) :
			rosNodeHandler(rosNodeHandler),
			topic (topic)
		{
			axesOutput.getSignal().clear();		// USER DEFINED clear all outputs
			buttonsOutput.getSignal().clear();

			subscriber = rosNodeHandler.subscribe(topic, queueSize, &ROSBlockTopic2::rosCallbackFct, this);
		}


		virtual void rosCallbackFct(const typeROSMessage& msg) {
			auto time = eeros::System::getTimeNs();

			// USER DEFINED: 1.) Set timestamp for all outputs
			//               2.) Get the data from the message
			//               3.) Cast the data if necessary
			//               4.) Insert the data into output

			axesOutput.getSignal().setTimestamp( time );
			buttonsOutput.getSignal().setTimestamp( time );

			std::vector<double> tmp( msg.axes.begin(), msg.axes.end() );	//cast because axes is a float32 vector
			axesValue.setCol(0, tmp);
			axesOutput.getSignal().setValue(axesValue);

			buttonsValue.setCol(0, msg.buttons);
			buttonsOutput.getSignal().setValue(buttonsValue);
			// END OF USER DEFINED
		}

		virtual void run() {
			ros::spinOnce();	//every run, ROS checks, if there is a new message on subscribed topic
		}


		// USER DEFINED: Add a 'getOut()' function for each output
		virtual eeros::control::Output<typeAxesOutput>& getAxesOut() {
			return axesOutput;
		}


		virtual eeros::control::Output<typeButtonsOutput>& getButtonsOut() {
			return buttonsOutput;
		}


	protected:
		// USER DEFINED: Necessary member variables
		eeros::control::Output<typeAxesOutput> axesOutput;
		typeAxesOutput axesValue;
		eeros::control::Output<typeButtonsOutput> buttonsOutput;
		typeButtonsOutput buttonsValue;

		//ROS variables
		ros::NodeHandle& rosNodeHandler;
		ros::Subscriber subscriber;
		const std::string& topic;



	private:
	};
}

#endif /* ORG_EEROS_CONTROL_ROSBLOCK_HPP_ */
