#ifndef ORG_EEROS_HAL_SBGELLIPSEA_HPP_
#define ORG_EEROS_HAL_SBGELLIPSEA_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <sbgEComLib.h>
#include <atomic>

using namespace eeros::math;
using namespace eeros::logger;

namespace eeros {
namespace hal {
  
/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::SBGEllipseAInput class. 
 * Do not use it directly.
 *
 * @since v1.3
 */
class SBGEllipseA : public eeros::Thread {
 public: 
  /**
   * Constructs a thread to get SBGEllipseA (IMU) sensors data \n
   *
   * @param dev - string with device name
   * @param priority - execution priority of thread to get sensors data
   */
  explicit SBGEllipseA(std::string dev, int priority) 
      : Thread(priority), starting(true), running(false), enableFastData(false), log(Logger::getLogger()) {
    auto errorCode = sbgInterfaceSerialCreate(&sbgInterface, dev.c_str(), 921600);
    log.info() << "SbgEllipseA: interfaceSerialCreate = " << errorCode;
    if (errorCode != SBG_NO_ERROR){
      log.error() << "SbgEllipseA: Unable to create serial interface";
      return; 
    }
    errorCode = sbgEComInit(&comHandle, &sbgInterface);
    if(errorCode != SBG_NO_ERROR){
      log.error() << "SbgEllipseA: Unable to initialize the sbgECom library";
      sbgInterfaceSerialDestroy(&sbgInterface);
    }
    errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);
    if (errorCode == SBG_NO_ERROR)
      log.info() << "SbgEllipseA: Device " << deviceInfo.serialNumber << " found";
    disableUnusedLogs(); 
    if (enableFastData){
      log.info() << "SbgEllipseA: Enabling IMU fast data rate";
      configureLogsHighRate();  // IMU data (1kHz) -> see "sbgEComCmdOutput.c"
    } else {
      log.info() << "SbgEllipseA: Enabling IMU 200 Hz data rate";
      configureLogsLowRate();   // IMU data, euler angles and quaternions (200Hz) -> see "sbgEComCmdOutput.c"
    }
    errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
    errorCode = sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL); // -> see: sbgECom.c
    starting = false;
  }
          
  /**
   * Destructs a the thread.
   */
  ~SBGEllipseA() {
    running = false; 
    join(); 
    sbgEComClose(&comHandle);
    sbgInterfaceSerialDestroy(&sbgInterface);
  }
    
  static Vector3 eulerData, accData, gyroData;
  static Vector4 quatData;
  static uint32_t timestampEuler, timestampQuat, timestampAcc, timestampGyro;
  static uint32_t count, count0, countEuler, countQuat, countImu;
  
 private:
  std::atomic<bool> starting;
  std::atomic<bool> running;
  bool enableFastData;
  Logger log;
  SbgEComHandle comHandle;
  SbgInterface sbgInterface;
  int32 retValue;
  SbgEComDeviceInfo deviceInfo;
    
  /**
   * Main run method. Gets sensor measurements \n
   */
  virtual void run() {
    while(starting);
    running = true;
    while (running) {
      sbgEComHandle(&comHandle);  // -> see: sbgECom.
      usleep(1000);
    }
  }
    
  /**
   * Disable logs which are not used. No data transmission for these log types \n
   */
  void disableUnusedLogs() {
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, SBG_ECOM_OUTPUT_MODE_DISABLED);     
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DISABLED); 
    sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DISABLED); 
  }
    
  /**
   * Configures log types for high rate data acquisition 
   */
  void configureLogsHighRate(){    
    auto errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_1, SBG_ECOM_LOG_FAST_IMU_DATA, SBG_ECOM_OUTPUT_MODE_HIGH_FREQ_LOOP); 
    if(errorCode != SBG_NO_ERROR) log.error() << "SbgEllipseA: Unable to configure output log SBG_ECOM_LOG_FAST_IMU_DATA";
    else log.info() << "SbgEllipseA: configured output log SBG_ECOM_LOG_FAST_IMU_DATA";
  }
    
  /**
   * Configures log types for slow rate data acquisition 
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
   * @param pHandle Valid handle on the sbgECom instance that has called this callback.
   * @param msgClass Class of the message we have received
   * @param msg Message ID of the log received.
   * @param pLogData Contains the received log data as an union.
   * @param pUserArg Optional user supplied argument.
   * @return SBG_NO_ERROR if the received log has been used successfully.
   */
  static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg) {  
    count++;
    switch (msg) {
      case SBG_ECOM_LOG_EKF_EULER:    
        eulerData << pLogData->ekfEulerData.euler[0],   
                     pLogData->ekfEulerData.euler[1],    
                     pLogData->ekfEulerData.euler[2];
        timestampEuler = pLogData->ekfEulerData.timeStamp;
        countEuler++;
        break;
      case SBG_ECOM_LOG_EKF_QUAT: 
        quatData << pLogData->ekfQuatData.quaternion[0],   
                    pLogData->ekfQuatData.quaternion[1],    
                    pLogData->ekfQuatData.quaternion[2],    
                    pLogData->ekfQuatData.quaternion[3];
        timestampQuat = pLogData->ekfQuatData.timeStamp;
        countQuat++;
        break;
      case SBG_ECOM_LOG_IMU_DATA: 
        accData << pLogData->imuData.accelerometers[0],   
                   pLogData->imuData.accelerometers[1],    
                   pLogData->imuData.accelerometers[2];
        timestampAcc = pLogData->imuData.timeStamp;
        gyroData << pLogData->imuData.gyroscopes[0],   
                    pLogData->imuData.gyroscopes[1],    
                    pLogData->imuData.gyroscopes[2];
        timestampGyro = pLogData->imuData.timeStamp;
        countImu++;
        break;
      case SBG_ECOM_LOG_FAST_IMU_DATA: // sbgEComBinaryLogImu.h
        accData << pLogData->fastImuData.accelerometers[0],   
                   pLogData->fastImuData.accelerometers[1],    
                   pLogData->fastImuData.accelerometers[2];
        timestampAcc = pLogData->fastImuData.timeStamp;
        gyroData << pLogData->fastImuData.gyroscopes[0],   
                    pLogData->fastImuData.gyroscopes[1],    
                    pLogData->fastImuData.gyroscopes[2];
        timestampGyro = pLogData->fastImuData.timeStamp;
        break;
     default:
       count0++;
       break;
   }
   return SBG_NO_ERROR;
 }
};

}
}

#endif /* ORG_EEROS_HAL_SBGELLIPSEA_HPP_ */

