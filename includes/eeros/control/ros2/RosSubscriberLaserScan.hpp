#ifndef ORG_EEROS_CONTROL_ROSSUBSCRIBER_LASERSCAN_HPP
#define ORG_EEROS_CONTROL_ROSSUBSCRIBER_LASERSCAN_HPP

#include <eeros/control/ros2/RosSubscriber.hpp>
#include <eeros/core/System.hpp>

// A-1 Include the header file of the ROS message
#include <sensor_msgs/LaserScan.h>

namespace eeros {
namespace control {

// C-1 Create the template definition. Each EEROS matrix output needs its own type
template < typename TRangesOutput, typename TIntensitiesOutput >
// A-3 Name your block and create the constructor. Copy the type definition
class RosSubscriberLaserScan : public RosSubscriber<sensor_msgs::LaserScan::Type, double> {
  // A-2 Define the type of the ROS message
  typedef sensor_msgs::LaserScan::Type	TRosMsg;
 public:
  RosSubscriberLaserScan(const std::string& topic, const uint32_t queueSize=1000, const bool callNewest=false ) :
    RosSubscriber<TRosMsg, double>(topic, queueSize, callNewest){ }
    
  void rosCallbackFct(const TRosMsg& msg) {
    // B-3 Set the timestamp of all EEROS signals
// 			auto time = eeros::System::getTimeNs();	// use system time for timestamp
    auto time = msg.header.stamp.toNSec();	// use received timestamp
    angle_minOutput.getSignal().setTimestamp( time );
    angle_maxOutput.getSignal().setTimestamp( time );
    angle_incrementOutput.getSignal().setTimestamp( time );
    time_incrementOutput.getSignal().setTimestamp( time );
    scan_timeOutput.getSignal().setTimestamp( time );
    range_minOutput.getSignal().setTimestamp( time );
    range_maxOutput.getSignal().setTimestamp( time );
    
    // B-4 Cast the data of the ROS message and fill it into the EEROS signals
    angle_minOutput.getSignal().setValue(		static_cast< double > ( msg.angle_min ) );
    angle_maxOutput.getSignal().setValue(		static_cast< double > ( msg.angle_max ) );
    angle_incrementOutput.getSignal().setValue(	static_cast< double > ( msg.angle_increment ) );
    time_incrementOutput.getSignal().setValue(	static_cast< double > ( msg.time_increment ) );
    scan_timeOutput.getSignal().setValue(		static_cast< double > ( msg.scan_time ) );
    range_minOutput.getSignal().setValue(		static_cast< double > ( msg.range_min ) );
    range_maxOutput.getSignal().setValue(		static_cast< double > ( msg.range_max ) );
    
    // C-4 Convert the vector of the ROS message to a <double>-vector
    std::vector<double> rangesTmp( msg.ranges.begin(), msg.ranges.end() );	//cast because axes is a float32 vector
    // C-5 Fill the vector into an EEROS matrix
    rangesValue.setCol(0, rangesTmp);
    // C-6 Fill the EEROS matrix in a EEROS signal
    rangesOutput.getSignal().setValue(rangesValue);
    
    std::vector<double> intensitiesTmp( msg.intensities.begin(), msg.intensities.end() );	//cast because axes is a float32 vector
    intensitiesValue.setCol(0, intensitiesTmp);
    intensitiesOutput.getSignal().setValue(intensitiesValue);
  }

  // B-2 Add a 'getOutput()' function for each output
  Output<double>& getAngle_minOutput()			{return angle_minOutput; }
  Output<double>& getAngle_maxOutput()			{return angle_maxOutput; }
  Output<double>& getAngle_incrementOutput()		{return angle_incrementOutput; }
  Output<double>& getTime_incrementOutput()		{return time_incrementOutput; }
  Output<double>& getScan_timeOutput()			{return scan_timeOutput; }
  Output<double>& getRange_minOutput()			{return range_minOutput; }
  Output<double>& getRange_maxOutput()			{return range_maxOutput; }
  // C-3 Add a 'getOutput()' function for each EEROS matrix output
  Output<TRangesOutput>& getRangesOutput()			{return rangesOutput; }
  Output<TIntensitiesOutput>& getIntensitiesOutput()	{return intensitiesOutput; }
  
 protected:
  //TODO Header
  // B-1 Create EEROS outputs
  Output<double>		angle_minOutput;
  Output<double>		angle_maxOutput;
  Output<double>		angle_incrementOutput;
  Output<double>		time_incrementOutput;
  Output<double>		scan_timeOutput;
  Output<double>		range_minOutput;
  Output<double>		range_maxOutput;
  // C-2 Create a 'value' and an 'output' variable for each EEROS matrix output
  TRangesOutput				rangesValue;
  Output<TRangesOutput>		rangesOutput;
  TIntensitiesOutput			intensitiesValue;
  Output<TIntensitiesOutput>	intensitiesOutput;
};

}
}

#endif // ORG_EEROS_CONTROL_ROSSUBSCRIBER_LASERSCAN_HPP
