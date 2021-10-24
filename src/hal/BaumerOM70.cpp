#include <eeros/hal/BaumerOM70.hpp>

using namespace eeros::hal;

BaumerOM70::BaumerOM70 (std::string dev, int port, int slaveId, int priority) 
    : Thread (priority), starting(true), running(false), log(Logger::getLogger()) {
  id = slaveId;
  ctx = modbus_new_tcp ( dev.c_str(), port ); // Create new tcp connection
  modbus_set_slave ( ctx, slaveId ); // Set slave
  log.info() << "Baumer OM70, Modbus new TCP connection " << dev.c_str() << ", slaveId " << slaveId;
  auto connectOutput = modbus_connect (ctx); // Connect
  if (connectOutput == -1) {
    log.info() << "Baumer OM70, Modbus connection failed: " << modbus_strerror ( errno );
    modbus_free(ctx);
    return;
  } else {
    log.info() << "Baumer OM70, Modbus connection successfull";
  }
  starting = false;
}

BaumerOM70::~BaumerOM70() {
  running = false;
  join();
  modbus_close(ctx);
  modbus_free(ctx);
}

void BaumerOM70::getMeasurements() {
  modbus_read_input_registers (ctx, 200, 17, tabReg);

  // Distance [mm]
  uint32_t* ptr_i = (uint32_t*) &tabReg[3];
  float* ptr_f = reinterpret_cast<float*> (ptr_i);

  distance = (*ptr_f) * 0.001; // from [mm] to [m]

  // Measurement rate [Hz]
//   uint32_t* ptr_i_rate = (uint32_t*) &tabReg[5];
//   float* ptr_rate = reinterpret_cast<float*> (ptr_i_rate);
//   measRate = *ptr_rate;

  // Timestamp [us]
//   uint32_t* ptr_i_stamp = (uint32_t*) &tabReg[15];
//   timestampUs = *ptr_i_stamp;
}

void BaumerOM70::getMeasRangeLimits() {
//   int rc = modbus_read_input_registers(ctx, 150, 6, tabReg);
}

void BaumerOM70::getIpAddress()     {
//   int rc = modbus_read_input_registers(ctx, 100, 6, tabReg); // Ethernet configuration
//   uint32_t* ptr_i = (uint32_t*) &tabReg[0];
//   float* ptr_f = reinterpret_cast<float*> (ptr_i); // TODO check to have string
}

void BaumerOM70::switchOnLaser() {
  log.info() << "Switch laser on";
  const uint16_t on = 1;
  int rc = modbus_write_register(ctx, 410, on);
  if (rc == -1) log.error() << "write error " << modbus_strerror ( errno );
  rc = modbus_read_registers(ctx, 410, 1, tabReg);
  if (rc == -1) log.error() << "read error " << modbus_strerror ( errno );
  else log.info() << tabReg[0];
}

void BaumerOM70::switchOffLaser() {
  const uint16_t off = 0;
  int rc = modbus_write_register(ctx, 410, off);
  if (rc == -1) log.error() << "write error " << modbus_strerror ( errno );
  rc = modbus_read_registers(ctx, 410, 1, tabReg);
  if (rc == -1) log.error() << "read error " << modbus_strerror ( errno );
  else log.info() << tabReg[0];
}

float BaumerOM70::getDistance() {
  return distance;
}

void BaumerOM70::run() {
  while(starting);
  running = true;
  while (running) {
    getMeasurements();
  }
}

