#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

#include <eeros/hal/SBGEllipseA.hpp>
// #include "../constants.hpp"

using namespace eeros::logger;
using namespace eeros::math;
using namespace eeros::hal;

Vector3 SBGEllipseA::data_euler;
Vector4 SBGEllipseA::data_quat;
Vector3 SBGEllipseA::data_acc;
Vector3 SBGEllipseA::data_gyro;
uint32_t SBGEllipseA::timestamp_euler;
uint32_t SBGEllipseA::timestamp_quat;
uint32_t SBGEllipseA::timestamp_acc;
uint32_t SBGEllipseA::timestamp_gyro;
uint32_t SBGEllipseA::count = 0;
uint32_t SBGEllipseA::count0 = 0;
uint32_t SBGEllipseA::countEuler = 0;
uint32_t SBGEllipseA::countQuat = 0;
uint32_t SBGEllipseA::countImu = 0;

SBGEllipseA::SBGEllipseA(std::string dev, int priority) : 
Thread(priority),
log(Logger::getLogger()),
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
//     log.info() << "SbgEllipseA: sbgEComInit = " << errorCode;
    
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

SBGEllipseA::~SBGEllipseA() {
	running = false; 
	join(); 
    
    sbgEComClose(&comHandle);
    sbgInterfaceSerialDestroy(&sbgInterface);
}

void SBGEllipseA::disableUnusedLogs() {
    auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DISABLED);     
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DISABLED); 
}

void SBGEllipseA::configureLogsHighRate() {    
    auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_1, SBG_ECOM_LOG_FAST_IMU_DATA, SBG_ECOM_OUTPUT_MODE_HIGH_FREQ_LOOP); 
    
    if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_FAST_IMU_DATA";
    else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_FAST_IMU_DATA";
}

void SBGEllipseA::configureLogsLowRate()   {
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

void SBGEllipseA::run() {
    while(!started);
    running = true;

    while (running) {
// 		std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

        auto errorCode = sbgEComHandle(&comHandle);  // -> see: sbgECom.
		
// 		double diff = std::chrono::duration<double, std::micro>(std::chrono::high_resolution_clock::now()-start).count();
// 		std::cout << diff << std::endl;
		
		usleep(1000);
    }
}

SbgErrorCode SBGEllipseA::onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, 
                                        const SbgBinaryLogData *pLogData, void *pUserArg)
{  
//     std::cout << "msg class " << (int)msgClass << std::endl;
//     std::cout << "log received " << (int)msg << std::endl;
	
// 	auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch());
// 	std::cout << now.count() << std::endl;
	
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
			
// 			std::cout << "data euler " << data_euler << std::endl;
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
