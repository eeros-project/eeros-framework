#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
// #include <libmodbus-3.1.6/src/modbus-tcp.h>
#include "../external/libmodbus-3.1.6/src/modbus-tcp.h"

#include <bits/stdc++.h> // TODO delete after clock test

#include <eeros/hal/BaumerOM70.hpp>
// #include "../constants.hpp"

using namespace eeros::logger;
using namespace eeros::math;


BaumerOM70::BaumerOM70(std::string dev, int port, int slave_id, int priority) : 
Thread(priority),
log(Logger::getLogger()) 
{       
    slaveId = slave_id;
    
    // Create new tcp connection
    ctx = modbus_new_tcp(dev.c_str(), port);
    
    // Set slave
    modbus_set_slave(ctx, slave_id);
    
    log.info() << "Baumer OM70, Modbus new TCP connection " << dev.c_str() << ", slaveId " << slaveId;
    
    // Connect
    auto connect_output = modbus_connect(ctx);
    
    if (connect_output == -1) {
        log.info() << "Baumer OM70, Modbus connection failed: " << modbus_strerror(errno);
        modbus_free(ctx);
        return;
    }
    else{
        log.info() << "Baumer OM70, Modbus connection successfull"; //: " << connect_output;
    }
    
    started = true; 
}


BaumerOM70::~BaumerOM70() {
	running = false; 
	join(); 
    
    modbus_close(ctx);
    modbus_free(ctx);
}

void BaumerOM70::run() {
    while(!started);
    running = true;

    while (running) {
        get_measurements();
    }
}

void BaumerOM70::get_measurements(){
    
    time_t start, end;
    pc.tick();
    
    start = clock();
    rc = modbus_read_input_registers(ctx, 200, 17, tab_reg);
    end = clock();
    
    pc.tock();
    
    // Distance [mm]
    uint32_t* ptr_i = (uint32_t*)&tab_reg[3];
    float* ptr_f = reinterpret_cast<float*>(ptr_i);
    
    distance = (*ptr_f)*0.001; // from [mm] to [m]
    
    // Measurement rate [Hz]
    uint32_t* ptr_i_rate = (uint32_t*)&tab_reg[5];
    float* ptr_rate = reinterpret_cast<float*>(ptr_i_rate);
    
    meas_rate = *ptr_rate;
    
    // Timestamp [us]
    uint32_t* ptr_i_stamp = (uint32_t*)&tab_reg[15];
    timestamp_us = *ptr_i_stamp;
}

void BaumerOM70::get_meas_range_limits(){
    rc = modbus_read_input_registers(ctx, 150, 6, tab_reg);
    
}

void BaumerOM70::get_ip_adress(){
    rc = modbus_read_input_registers(ctx, 100, 6, tab_reg); // Ethernet configuration
    uint32_t* ptr_i = (uint32_t*)&tab_reg[0];
    float* ptr_f = reinterpret_cast<float*>(ptr_i); // TODO check to have string
}

void BaumerOM70::switch_on_laser(){
    log.info() << "Switch on";
    const uint16_t on = 1;
    rc = modbus_write_register(ctx, 410, on); // TODO write error Illegal data value
    
    if (rc == -1) {
        log.error() << "write error " << modbus_strerror(errno);
    }
    
    rc = modbus_read_registers(ctx, 410, 1, tab_reg);
    if (rc == -1)
        log.error() << "read error " << modbus_strerror(errno);
    else
        log.info() << tab_reg[0];
}

void BaumerOM70::switch_off_laser(){
    const uint16_t off = 0;
    rc = modbus_write_register(ctx, 410, off); // TODO write error Illegal data value
    
    if (rc == -1) {
        log.error() << "write error " << modbus_strerror(errno);
    }
    
    rc = modbus_read_registers(ctx, 410, 1, tab_reg);
    if (rc == -1)
        log.error() << "read error " << modbus_strerror(errno);
    else
        log.info() << tab_reg[0];
}

float BaumerOM70::get_distance(){
    return distance;
}
