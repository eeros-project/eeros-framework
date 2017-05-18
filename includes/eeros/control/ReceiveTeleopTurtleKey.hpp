#ifndef ORG_EEROS_CONTROL_RECEIVETELEOPTURTLEKEY_HPP_
#define ORG_EEROS_CONTROL_RECEIVETELEOPTURTLEKEY_HPP_

#include <string>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/core/System.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eeros/math/Matrix.hpp>


//using namespace eeros::math;
//using namespace eeros::hal;

namespace eeros {
	namespace control {
		typedef eeros::math::Matrix< 3 , 1 , double >	typeLinear;
		typedef eeros::math::Matrix< 3 , 1 , double >	typeAngular;

		class ReceiveTeleopTurtleKey: public Block {
		public:
			ReceiveTeleopTurtleKey(ros::NodeHandle& rosNodeHandler);
//			ReceiveTeleopTurtleKey(std::string& name);
			virtual ~ReceiveTeleopTurtleKey();

			virtual void run();

			virtual Output<typeLinear>& getLinear()		{ return linearSignal; };
			virtual Output<typeAngular>& getAngular()	{ return angularSignal; };

		protected:
			ros::NodeHandle& rosNodeHandler;
			ros::Subscriber sub;
			void rosCallback(const geometry_msgs::Twist::ConstPtr& vel);
			geometry_msgs::Twist vel_buffered;

			typeLinear				linearValue;
			Output<typeLinear>		linearSignal;
			typeAngular				angularValue;
			Output<typeAngular>		angularSignal;
		private:
		};
	};
};

#endif /* ORG_EEROS_CONTROL_RECEIVETELEOPTURTLEKEY_HPP_ */
