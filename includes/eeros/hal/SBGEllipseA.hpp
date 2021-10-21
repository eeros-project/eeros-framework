#ifndef ORG_EEROS_HAL_SBGELLIPSEA_HPP_
#define ORG_EEROS_HAL_SBGELLIPSEA_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <sbgEComLib.h>

// eeros::math::Vector3 SBGEllipseA::data_euler;
// eeros::math::Vector4 SBGEllipseA::data_quat;
// eeros::math::Vector3 SBGEllipseA::data_acc;
// eeros::math::Vector3 SBGEllipseA::data_gyro;
// uint32_t SBGEllipseA::timestamp_euler;
// uint32_t SBGEllipseA::timestamp_quat;
// uint32_t SBGEllipseA::timestamp_acc;
// uint32_t SBGEllipseA::timestamp_gyro;
// uint32_t SBGEllipseA::count = 0;
// uint32_t SBGEllipseA::count0 = 0;
// uint32_t SBGEllipseA::countEuler = 0;
// uint32_t SBGEllipseA::countQuat = 0;
// uint32_t SBGEllipseA::countImu = 0;

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
			explicit SBGEllipseA(std::string dev, int priority) : 
			Thread(priority),
			log(eeros::logger::Logger::getLogger()),
			enable_fast_data(false) 
			{	
				// Open serial device
				auto errorCode = sbgInterfaceSerialCreate(&sbgInterface, dev.c_str(), 921600);
				log.info() << "SbgEllipseA: interfaceSerialCreate = " << errorCode;
				
				if(errorCode != SBG_NO_ERROR){
					log.error() << "SbgEllipseA: Unable to create serial interface";
					return; 
				}
				
				// Open library
				errorCode = sbgEComInit(&comHandle, &sbgInterface);
				
				if(errorCode != SBG_NO_ERROR){
					log.error() << "SbgEllipseA: Unable to initialize the sbgECom library";
					
					// Close serial interface
					sbgInterfaceSerialDestroy(&sbgInterface);
				}
				
				// Get Device Infos
				errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);
				
				if(errorCode == SBG_NO_ERROR)
					log.info() << "SbgEllipseA: Device " << deviceInfo.serialNumber << " found";
				
				// Configure Logs
				disableUnusedLogs(); 
				
				if(enable_fast_data){
					log.info() << "SbgEllipseA: Enabling IMU fast data rate";
					configureLogsHighRate();  // IMU data (1kHz) -> see "sbgEComCmdOutput.c"
				}
				else{
					log.info() << "SbgEllipseA: Enabling IMU 200 Hz data rate";
					configureLogsLowRate();   // IMU data, euler angles and quaternions (200Hz) -> see "sbgEComCmdOutput.c"
				}
				
				// Save settings
				errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);

				// Set callbacks
				errorCode = sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL); // -> see: sbgECom.c
				
				// Start 
				started = true;
			}
						
			/**
			* Destructs a Thread to get SBGEllipseA sensors data \n
			*/
			~SBGEllipseA() {
				running = false; 
				join(); 
				
				sbgEComClose(&comHandle);
				sbgInterfaceSerialDestroy(&sbgInterface);
			}
			
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
			volatile bool started = false;
			volatile bool running = false;
			bool enable_fast_data;
			
			SbgEComHandle			comHandle;
			SbgInterface			sbgInterface;
			int32					retValue;
			SbgEComDeviceInfo		deviceInfo;
			
			/**
			* Main run method. Gets sensor measurements \n
			*/
			virtual void run() {
				while(!started);
				running = true;

				while (running) {
					auto errorCode = sbgEComHandle(&comHandle);  // -> see: sbgECom.
					usleep(1000);
				}
			}
			
			/**
			* Disable Logs which are not used. No data transmission for these log types \n
			*/
			void disableUnusedLogs() {
				auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DISABLED); 
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DISABLED);     
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, SBG_ECOM_OUTPUT_MODE_DISABLED); 
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DISABLED); 
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DISABLED); 
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DISABLED); 
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DISABLED); 
			}
			
			/**
			* Configures Log Types for high rate data acquisition 
			*/
			void configureLogsHighRate(){    
				auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_1, SBG_ECOM_LOG_FAST_IMU_DATA, SBG_ECOM_OUTPUT_MODE_HIGH_FREQ_LOOP); 
				
				if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_FAST_IMU_DATA";
				else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_FAST_IMU_DATA";
			}
			
			/**
			* Configures Log Types for slow rate data acquisition 
			*/
			void configureLogsLowRate() {
				auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, 
														SBG_ECOM_OUTPUT_MODE_MAIN_LOOP); 
					
				if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_IMU_DATA";
				else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_IMU_DATA";
				
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER,
														SBG_ECOM_OUTPUT_MODE_MAIN_LOOP);  
				
				if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_EKF_EULER";
				else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_EKF_EULER";
				
				errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT,
														SBG_ECOM_OUTPUT_MODE_MAIN_LOOP);  
				
				if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_EKF_QUAT";
				else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_EKF_QUAT";
			}
			
			/**
			* Callback definition called each time a new log is received.
			* @param	pHandle		Valid handle on the sbgECom instance that has called this callback.
			* @param	msgClass	Class of the message we have received
			* @param	msg			Message ID of the log received.
			* @param	pLogData	Contains the received log data as an union.
			* @param	pUserArg	Optional user supplied argument.
			* @return				SBG_NO_ERROR if the received log has been used successfully.
			*/
			static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg){  
					count++;
					// Handle separately each received data according to the log ID
					switch (msg)
					{
						case SBG_ECOM_LOG_EKF_EULER:    
							data_euler     << pLogData->ekfEulerData.euler[0],   
											pLogData->ekfEulerData.euler[1],    
											pLogData->ekfEulerData.euler[2];
							timestamp_euler = pLogData->ekfEulerData.timeStamp;
							countEuler++;
							
							break;
							
					case SBG_ECOM_LOG_EKF_QUAT: 
						data_quat     << pLogData->ekfQuatData.quaternion[0],   
											pLogData->ekfQuatData.quaternion[1],    
											pLogData->ekfQuatData.quaternion[2],    
											pLogData->ekfQuatData.quaternion[3];
						timestamp_quat = pLogData->ekfQuatData.timeStamp;
						countQuat++;
						break;
					
					case SBG_ECOM_LOG_IMU_DATA: 
						data_acc     << pLogData->imuData.accelerometers[0],   
										pLogData->imuData.accelerometers[1],    
										pLogData->imuData.accelerometers[2];
						timestamp_acc = pLogData->imuData.timeStamp;
										
						data_gyro     << pLogData->imuData.gyroscopes[0],   
											pLogData->imuData.gyroscopes[1],    
											pLogData->imuData.gyroscopes[2];
						timestamp_gyro = pLogData->imuData.timeStamp;
							countImu++;
							break;
										
					case SBG_ECOM_LOG_FAST_IMU_DATA: // sbgEComBinaryLogImu.h
						data_acc     << pLogData->fastImuData.accelerometers[0],   
										pLogData->fastImuData.accelerometers[1],    
										pLogData->fastImuData.accelerometers[2];
						timestamp_acc = pLogData->fastImuData.timeStamp;
										
						data_gyro     << pLogData->fastImuData.gyroscopes[0],   
											pLogData->fastImuData.gyroscopes[1],    
											pLogData->fastImuData.gyroscopes[2];
						timestamp_gyro = pLogData->fastImuData.timeStamp;
						break;
						
						default:
							count0++;
							break;
					}
					
					return SBG_NO_ERROR;
				}
	};
};
}

#endif /* ORG_EEROS_HAL_SBGELLIPSEA_HPP_ */

