#ifndef ORG_EEROS_HAL_SBGELLIPSEA_HPP_
#define ORG_EEROS_HAL_SBGELLIPSEA_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>

#include <sbgEComLib.h>

namespace eeros {
namespace hal {
		
	/**
	* This class is part of the hardware abstraction layer. 
	* It is used by \ref eeros::control::SBGEllipseAInput class. 
	* Do not use it directly.
	*
	*/
	class SBGEllipseA : public eeros::Thread {
		public: 
			/**
			* Constructs a Thread to get SBGEllipseA (IMU) sensors data \n
			* Calls SBGEllipseA(std::string dev, int priority)
			*
			* @see SBGEllipseA(std::string dev, int priority)
			* @param dev - string with device name
			* @param priority - execution priority or BaumerOM70 thread, to get sensors data
			*/
			explicit SBGEllipseA(std::string dev, int priority);
						
			/**
			* Destructs a Thread to get SBGEllipseA sensors data \n
			*/
			~SBGEllipseA();
			
			static eeros::math::Vector3 data_euler;
			static eeros::math::Vector4 data_quat;
			static eeros::math::Vector3 data_acc;
			static eeros::math::Vector3 data_gyro;
			static uint32_t timestamp_euler;
			static uint32_t timestamp_quat;
			static uint32_t timestamp_acc;
			static uint32_t timestamp_gyro;
			
			static uint32_t count, count0, countEuler, countQuat, countImu;
		private:
			eeros::logger::Logger log;
			virtual void run();
			volatile bool started = false;
			volatile bool running = false;
			
			bool enable_fast_data;
			SbgEComHandle			comHandle;
			SbgInterface			sbgInterface;
			int32					retValue;
			SbgEComDeviceInfo		deviceInfo;
			
			/**
			* Disable Logs which are not used. No data transmission for these log types \n
			*/
			void disableUnusedLogs();
			
			/**
			* Configures Log Types for high rate data acquisition 
			*/
			void configureLogsHighRate();
			
			/**
			* Configures Log Types for slow rate data acquisition 
			*/
			void configureLogsLowRate();
			
			/**
			* Callback definition called each time a new log is received.
			* @param	pHandle		Valid handle on the sbgECom instance that has called this callback.
			* @param	msgClass	Class of the message we have received
			* @param	msg			Message ID of the log received.
			* @param	pLogData	Contains the received log data as an union.
			* @param	pUserArg	Optional user supplied argument.
			* @return				SBG_NO_ERROR if the received log has been used successfully.
			*/
			static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);
	};
};
}

#endif /* ORG_EEROS_HAL_SBGELLIPSEA_HPP_ */

