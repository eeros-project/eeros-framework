#ifndef ORG_EEROS_CONTROL_ROSPUBLISHER_LASERSCAN_HPP
#define ORG_EEROS_CONTROL_ROSPUBLISHER_LASERSCAN_HPP

#include <eeros/control/ROS/RosPublisher.hpp>
#include <eeros/math/Matrix.hpp>

// A-1 Include the header file of the ROS message
#include <sensor_msgs/LaserScan.h>



namespace eeros {
	namespace control {
		// C-1 Create the template definition. Each EEROS matrix input needs its own type
		template < typename TRangesInput, typename TIntensitiesInput >
		// A-3 Name your block and create the constructor. Copy the type definition
		class RosPublisherLaserScan : public RosPublisher<sensor_msgs::LaserScan::Type, double> {
			// A-2 Define the type of the ROS message
			typedef sensor_msgs::LaserScan::Type	TRosMsg;
		public:
			RosPublisherLaserScan(ros::NodeHandle& rosNodeHandler, const std::string& topic, const uint32_t queueSize=1000) :
				RosPublisher<TRosMsg, double>(rosNodeHandler, topic, queueSize) { }
				
			void setRosMsg(TRosMsg& msg) {
				// B-3 If available, set time in msg header
				msg.header.stamp = eeros::control::rosTools::convertToRosTime(eeros::System::getTimeNs());
				
				// B-4 Check if EEROS input is connected. Cast the data. Assign casted data to ROS message field
				if (angle_minInput.isConnected() )
					msg.angle_min		= static_cast<float>( angle_minInput.getSignal().getValue() );
				if (angle_maxInput.isConnected() )
					msg.angle_max 		= static_cast<float>( angle_maxInput.getSignal().getValue() );
				if (angle_incrementInput.isConnected() )
					msg.angle_increment = static_cast<float>( angle_incrementInput.getSignal().getValue() );
				if (time_incrementInput.isConnected() )
					msg.time_increment	= static_cast<float>( time_incrementInput.getSignal().getValue() );
				if (scan_timeInput.isConnected() )
					msg.scan_time		= static_cast<float>( scan_timeInput.getSignal().getValue() );
				if (range_minInput.isConnected() )
					msg.range_min		= static_cast<float>( range_minInput.getSignal().getValue() );
				if (range_maxInput.isConnected() )
					msg.range_max		= static_cast<float>( range_maxInput.getSignal().getValue() );
				// C-4 Check if EEROS input is connected
				if (rangesInput.isConnected() ) {
					// C-5 Get the vector from the EEROS input
					rangesValue = rangesInput.getSignal().getValue();
					auto rangesTmpDouble = rangesValue.getColVector(0);
					// C-6 Cast the vector and assign it to the appropriate ROS data field
					std::vector<float> rangesTmpFloat( rangesTmpDouble.begin(), rangesTmpDouble.end() );	// cast to float vector
					msg.ranges			= rangesTmpFloat;
				}
				if (intensitiesInput.isConnected() ) {
					intensitiesValue = intensitiesInput.getSignal().getValue();
					auto intensitiesTmpDouble = rangesValue.getColVector(0);
					std::vector<float> intensitiesTmpFloat( intensitiesTmpDouble.begin(), intensitiesTmpDouble.end() );	// cast to float vector
					msg.intensities		= intensitiesTmpFloat;
				}
			}
			
			// B-2 Add a 'getInput()' function for each input
			Input<double>& getAngle_minInput()			{return angle_minInput; };
			Input<double>& getAngle_maxInput()			{return angle_maxInput; };
			Input<double>& getAngle_incrementInput()	{return angle_incrementInput; };
			Input<double>& getTime_incrementInput()	{return time_incrementInput; };
			Input<double>& getScan_timeInput()			{return scan_timeInput; };
			Input<double>& getRange_minInput()			{return range_minInput; };
			Input<double>& getRange_maxInput()			{return range_maxInput; };
			// C-3 Add a 'getInput()' function for each EEROS matrix Input
			Input<TRangesInput>& getRangesInput()				{return rangesInput; };
			Input<TIntensitiesInput>& getIntensitiessInput()	{return intensitiesInput; };
				
		protected:
			// B-1 Create EEROS inputs
			Input<double> angle_minInput;
			Input<double> angle_maxInput;
			Input<double> angle_incrementInput;
			Input<double> time_incrementInput;
			Input<double> scan_timeInput;
			Input<double> range_minInput;
			Input<double> range_maxInput;
			// C-2 Create a 'value' and an 'input' variable for each EEROS matrix input
			TRangesInput				rangesValue;
			Input<TRangesInput>			rangesInput;
			TIntensitiesInput			intensitiesValue;
			Input<TIntensitiesInput>	intensitiesInput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_ROSPUBLISHER_LASERSCAN_HPP