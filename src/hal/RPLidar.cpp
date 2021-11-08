#include <eeros/hal/RPLidar.hpp>
// #include <eeros/control/TimeDomain.hpp>
// #include <eeros/logger/Logger.hpp>
// #include <eeros/core/Thread.hpp>
// #include <eeros/math/Matrix.hpp>
// 
// #define LASER_COUNT_MAX 380
// 

using namespace eeros::hal;
// 
// using namespace rp::standalone::rplidar;
// 
RPLidar::RPLidar(std::string dev, int priority) 
    : Thread(priority), starting(true), running(false), log(Logger::getLogger('P')) {
  bufSize = LASER_COUNT_MAX;
  ld = RPlidarDriver::CreateDriver(CHANNEL_TYPE_SERIALPORT);
  if (!ld){
    log.error() << "RPLidar Create driver instance: insufficent memory, exit";
    return;
  }
  rplidar_response_device_info_t devinfo;
  u_result res = ld->connect(dev.c_str(), 115200);
  if (IS_OK(res)) {    
    // Print out the device serial number, firmware and hardware version number
    printf("RPLidar S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
      printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n"
        "RPLidar Firmware Ver: %d.%02d\n"
        "RPLidar Hardware Rev: %d\n"
        , devinfo.firmware_version>>8
        , devinfo.firmware_version & 0xFF
        , (int)devinfo.hardware_version);

    //  Check health
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = ld->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
      log.info() <<"RPLidar health status : " << healthinfo.status;
      if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        log.error() << "RPLidar Error, rplidar internal error detected. Please reboot the device to retry";
      } 
    } else {
      log.error() << "RPLidar Error, cannot retrieve the lidar health code: " << op_result;
    }
  } else{
    log.error() << "RPLidar Failed to connect to LIDAR";
    return;
  }
  ld->startMotor();
  RplidarScanMode mode;
  ld->startScan(false, true, 0, &mode);  
  starting = false;
}

RPLidar::~RPLidar() { 
  running = false;
  join();
  ld->stop();
  ld->stopMotor();
  ld->disconnect();
  RPlidarDriver::DisposeDriver(ld);
}

float RPLidar::getScanFrequency(){
  float frequency = 0;
//  ld->getFrequency(scan_mode, bufSize, frequency);
  return frequency;
}

Vector<LASER_COUNT_MAX,double> RPLidar::getAngles() {
  return angles;
}

Vector<LASER_COUNT_MAX,double> RPLidar::getRanges() {
  return ranges;
}

Vector<LASER_COUNT_MAX,double> RPLidar::getIntensities() {
  return intensities;
}

void RPLidar::run() {
  while(starting);
  running = true;
  while (running) {
    u_result ans;
    rplidar_response_measurement_node_hq_t nodes[LASER_COUNT_MAX];
    size_t count = _countof(nodes);
    // fetch extactly one 0-360 degrees' scan
    ans = ld->grabScanDataHq(nodes, count);
    bufSize = count; 
    if (IS_OK(ans)){  
      ld->ascendScanData(nodes, count);
      // Set angles
//      int start_node = 0, end_node = 0;
      int i = 0;
      // find the first valid node and last valid node
      while (nodes[i++].dist_mm_q2 == 0);
//       start_node = i-1;
      i = count -1;
      while (nodes[i--].dist_mm_q2 == 0);
//       end_node = i+1;
      
      // Set ranges and intensities
      ranges.zero();
      intensities.zero();
      
      for (int pos = 0; pos < (int)count ; ++pos) {               
        float read_value = (float) nodes[pos].dist_mm_q2/4.0f/1000;
        if (fabs(read_value) < 0.01)
          ranges[pos] = std::numeric_limits<float>::infinity();
        else
          ranges[pos] = read_value;
        intensities[pos] = (float) (nodes[pos].quality >> 2);
        angles[pos] = (nodes[pos].angle_z_q14 * 90.f / 16384.f) * 3.1415926535/180;
      }
      // Set to infinite all elements of the fixed-sized vector for control system
      for (int pos = (int)count; pos < LASER_COUNT_MAX; ++pos) {  
        ranges[pos] = std::numeric_limits<float>::infinity();
        intensities[pos] = std::numeric_limits<float>::infinity();
        angles[pos] = std::numeric_limits<float>::infinity();
      }
    } else if(ans == RESULT_OPERATION_TIMEOUT){
      log.error() << "RPLidar Laser Timeout! error code: " << ans;
    } else {
      log.error() << "RPLidar Laser error code: " << ans;
    }
  }
}
