#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/hal/ODrive_USB.hpp>
#include <eeros/hal/constants_odrive.hpp>

#include <iostream>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <iomanip>
#include <unistd.h>
#include <math.h>

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CTRL_MODE_VELOCITY_CONTROL 2
#define CTRL_MODE_POSITION_CONTROL 3 

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::math;

ODrive_USB::ODrive_USB(uint64_t odrive_serialnr, float enc_ticks, int priority, bool first_drive) : 
odrive_serialnr(odrive_serialnr),
enc_ticks(enc_ticks),
first_drive(first_drive),
Thread(priority),
log(Logger::getLogger('P'))  {
	
	// initializations
	libusbContext = NULL;
    ep = new ODriveEP();

	int res = init(odrive_serialnr);
	if (res != 0) {
		log.error() << std::hex << "odrive with serial number 0x" << odrive_serialnr << " not found";
// 		return EXIT_FAILURE;
	}
	
	config_odrive();
	started = true;
}

ODrive_USB::~ODrive_USB() {
	// Set speed to zero
	ep->setData(param_m0_input_vel, (float)0.0); // motor0 vel_setpoint 
	ep->setData(param_m1_input_vel, (float)0.0); // motor1 vel_setpoint

	disable_drives();
	disable_watchdog();
	
	delete ep;
	if (libusbContext) { 
		libusb_exit(libusbContext); 
		libusbContext = NULL;
	}
		
	running = false;
}

void ODrive_USB::config_odrive() {
	// Disable Watchdog for initial settings (watchdog_feed() still not running)
	disable_watchdog();
	
	// Clear errors
	ep->execFunc(fcn_m0_clear_errors);
	ep->execFunc(fcn_m1_clear_errors);
	
	// Watchdog settings
	float timeout = 1.0;
	ep->setData(param_a0_watchdog_timeout, timeout);
	ep->setData(param_a1_watchdog_timeout, timeout);
	
	// Set endpoints (emergency button)
	ep->setData(param_m0_endstop_gpio_num, 5);
	ep->setData(param_m0_endstop_gpio_num, 6);
	ep->setData(param_m0_endstop_is_active_high, false);
	ep->setData(param_m0_endstop_is_active_high, false);
	ep->setData(param_m0_endstop_enabled, true);
	ep->setData(param_m1_endstop_enabled, true);
	
	// Set velocity control mode
	ep->setData(param_m0_control_mode, CTRL_MODE_VELOCITY_CONTROL);
	ep->setData(param_m1_control_mode, CTRL_MODE_VELOCITY_CONTROL);
	
	// Set controller gains
	set_controller_gains();
	
	// Save configuration
// 	ep->execFunc(fcn_save_configuration); // TODO still not working
}

void ODrive_USB::enable_watchdog() {
	ep->setData(param_a0_enable_watchdog, 1);
	ep->setData(param_a1_enable_watchdog, 1);
}

void ODrive_USB::disable_watchdog() {
	ep->setData(param_a0_enable_watchdog, 0);
	ep->setData(param_a1_enable_watchdog, 0);
}

void ODrive_USB::set_velocity_ctrl_mode(){
	ep->setData(param_m0_control_mode, CTRL_MODE_VELOCITY_CONTROL);
	ep->setData(param_m1_control_mode, CTRL_MODE_VELOCITY_CONTROL);
}

void ODrive_USB::set_controller_gains(){
	float vel_gain = 0.45;
	float pos_gain = 10.0;
	float vel_integrator_gain = 2.25;
	
	ep->setData(param_m0_vel_gain, vel_gain);
	ep->setData(param_m0_pos_gain, pos_gain);
	ep->setData(param_m0_vel_integrator_gain, vel_integrator_gain);
	ep->setData(param_m1_vel_gain, vel_gain);
	ep->setData(param_m1_pos_gain, pos_gain);
	ep->setData(param_m1_vel_integrator_gain, vel_integrator_gain);
}

int ODrive_USB::init(uint64_t odrive_serialnr){
  if (libusbContext != NULL) {
    std::cerr << "ODrive init function has been called twice." << std::endl;
    return 1;
  }

  int res = libusb_init(&libusbContext);
  if (res != LIBUSB_SUCCESS) {
    std::cerr << "could not initialize usb" << std::endl;
    return res;
  }

  libusb_device ** usb_device_list;

  ssize_t device_count = libusb_get_device_list(libusbContext, &usb_device_list);
  if (device_count <= 0) {
    return device_count;
  }
  
  // Reset device
  if(first_drive) {
	for (size_t i = 0; i < device_count; ++i) {
		libusb_device *device = usb_device_list[i];
		libusb_device_descriptor desc = {0};

		res = libusb_get_device_descriptor(device, &desc);
		if (res != LIBUSB_SUCCESS) {
		std::cerr << "error getting device descriptor" << std::endl;
		continue;
		}
		
		/* Check USB device ID */
		if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {	  
			libusb_device_handle *device_handle;
			if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
				std::cerr << "error opening USB device" << std::endl;
				continue;
			}
				
			libusb_reset_device(device_handle);
			std::cout << "reset device" << std::endl;
			sleep(1);
			libusb_close(device_handle);
		}
	} 
  }
  
  int ret = 1;
  for (size_t i = 0; i < device_count; ++i) {
    libusb_device *device = usb_device_list[i];
    libusb_device_descriptor desc = {0};

    res = libusb_get_device_descriptor(device, &desc);
    if (res != LIBUSB_SUCCESS) {
      std::cerr << "error getting device descriptor" << std::endl;
      continue;
    }
    
    /* Check USB device ID */
    if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {	  
      libusb_device_handle *device_handle;
      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
        std::cerr << "error opening USB device" << std::endl;
        continue;
      }
      
//       if(first_drive) {
// 		libusb_reset_device(device_handle);
// 		std::cout << "reset device" << std::endl;
// 		sleep(1);
// 	  }

      struct libusb_config_descriptor *config;
      res = libusb_get_config_descriptor(device, 0, &config);
      int ifNumber = 2; // config->bNumInterfaces;

      if ((libusb_kernel_driver_active(device_handle, ifNumber) != LIBUSB_SUCCESS) && 
        (libusb_detach_kernel_driver(device_handle, ifNumber) != LIBUSB_SUCCESS)) {
        std::cerr << "driver error" << std::endl;
        libusb_close(device_handle);
        continue;
      }
      
      if ((res = libusb_claim_interface(device_handle, ifNumber)) !=  LIBUSB_SUCCESS) {
        std::cerr << "error claiming device" << std::endl;
        libusb_close(device_handle);
        continue;
      } else {
        bool attached_to_handle = false;
        unsigned char buf[128];
        res = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber, buf, 127);
        if (res <= 0) {
          std::cerr << "error getting serial number data" << std::endl;
          res = libusb_release_interface(device_handle, ifNumber);
          libusb_close(device_handle);
          continue;
        } else {
          std::stringstream strea;
          strea << std::uppercase << std::hex << odrive_serialnr;
          std::string sn(strea.str());
          if (sn.compare(0, strlen((const char*)buf), (const char*)buf) == 0) {
            std::cout << "odrive device with serial number 0x" << sn << " found" << std::endl; 
            ep->devHandle = device_handle;
            attached_to_handle = true;
            ret = 0;
            break;
          }       
        }
        if (!attached_to_handle) {
          res = libusb_release_interface(device_handle, ifNumber);
          libusb_close(device_handle);
        }
      }
    }
  }
  libusb_free_device_list(usb_device_list, 1);
    
  return ret;
}

void ODrive_USB::calibrate_motors(){
	bool m0_calibrated, m0_is_encoder_ready;
	bool m1_calibrated, m1_is_encoder_ready;
	float inductance0, resistance0, inductance1, resistance1;
	
	// Clear motor errors
	ep->execFunc(fcn_m0_clear_errors);
	ep->execFunc(fcn_m1_clear_errors);
	
	// Do motors calibration, if inductance or resistance are not valid
	ep->getData(param_m0_phase_inductance, inductance0);
	ep->getData(param_m0_phase_resistance, resistance0);
	ep->getData(param_m1_phase_inductance, inductance1);
	ep->getData(param_m1_phase_resistance, resistance1);
	ep->getData(param_a0_is_calibrated, m0_calibrated);
	ep->getData(param_a1_is_calibrated, m1_calibrated);
	ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
	ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
	
	log.info()  << odrive_serialnr << " -> M0: inductance = " << inductance0 << ", resistance = " << resistance0 
				<< ", calibrated? " << m0_calibrated << ", encoder ready? " << m0_is_encoder_ready;
				
	log.info()  << odrive_serialnr << " -> M1: inductance = " << inductance1 << ", resistance = " << resistance1 
				<< ", calibrated? " << m1_calibrated << ", encoder ready? " << m1_is_encoder_ready;
	
	// Motors calibration
	if(isinf(inductance0) || isinf(resistance0) || !m0_calibrated) {
		log.info() << odrive_serialnr << " -> Performing M0 calibration ... ";
		ep->setData(param_a0_requested_state, AXIS_STATE_MOTOR_CALIBRATION);
		sleep(5);
	}
	if(isinf(inductance1) || isinf(resistance1) || !m1_calibrated) {
		log.info() << odrive_serialnr << " -> Performing M1 calibration ... ";
		ep->setData(param_a1_requested_state, AXIS_STATE_MOTOR_CALIBRATION);
		sleep(5);
	}
	
	ep->getData(param_a0_is_calibrated, m0_calibrated);
	ep->getData(param_a1_is_calibrated, m1_calibrated);
	log.info() << odrive_serialnr << " -> Motor calibrated: M0 -> " << m0_calibrated << "; M1 -> " << m1_calibrated;
	
	// Encoder calibration
	ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
	if(!m0_is_encoder_ready) {
		log.info() << odrive_serialnr << " -> Start M0 encoder offset calibration ...";
		ep->setData(param_a0_requested_state, AXIS_STATE_ENCODER_OFFSET_CALIBRATION); 
	}
	
	ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
	if(!m1_is_encoder_ready) {
		log.info() << odrive_serialnr << " -> Start M1 encoder offset calibration ...";
		ep->setData(param_a1_requested_state, AXIS_STATE_ENCODER_OFFSET_CALIBRATION); 
	}
	
	// Wait calibration to finish
	ep->getData(param_a0_is_calibrated, m0_calibrated);
	ep->getData(param_a1_is_calibrated, m1_calibrated);
	
	while (!m0_calibrated || !m0_is_encoder_ready || !m1_calibrated || !m1_is_encoder_ready){
		ep->getData(param_a0_is_calibrated, m0_calibrated);
		ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
		ep->getData(param_a1_is_calibrated, m1_calibrated);
		ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
// 		log.info() << 	"calibrating M0: " << m0_calibrated << " - " << m0_is_encoder_ready << 
// 						"; M1 " << m1_calibrated << " - " << m1_is_encoder_ready;
		sleep(1);
	}
	
	ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
	ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
	log.info() << odrive_serialnr << " -> Motor calibrated: M0 -> " << m0_is_encoder_ready << "; M1 -> " << m1_is_encoder_ready;
	
	// Save configuration TODO, not working
// 	ep->execFunc(save_configuration);
// 	log.info() << "Configuration saved";
	
	calibration_done = true;
}

void ODrive_USB::enable_drives(){
	ep->setData(param_a0_requested_state, AXIS_STATE_CLOSED_LOOP_CONTROL);
	ep->setData(param_a1_requested_state, AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void ODrive_USB::disable_drives(){
	ep->setData(param_a0_requested_state, AXIS_STATE_IDLE);
	ep->setData(param_a1_requested_state, AXIS_STATE_IDLE);
}

bool ODrive_USB::is_calibrated(){
	return calibration_done; 
}

void ODrive_USB::get_vel_from_odrive(){
	float vel0, vel1;
	ep->getData(param_m0_enc_vel_estimate, vel0); // encoder, vel_estimate
	ep->getData(param_m1_enc_vel_estimate, vel1); // encoder, vel_estimate
	
	encoder_vel0 = vel0;
	encoder_vel1 = vel1;
}

void ODrive_USB::get_current_from_odrive(){
	float iq_m_0, iq_s_0, iq_m_1, iq_s_1;
	ep->getData(param_m0_iq_measured, iq_m_0); 
	ep->getData(param_m0_iq_setpoint, iq_s_0); 
	ep->getData(param_m1_iq_measured, iq_m_1); 
	ep->getData(param_m1_iq_setpoint, iq_s_1); 
	
	iq_measured0 = iq_m_0;
	iq_setpoint0 = iq_s_0;
	iq_measured1 = iq_m_1;
	iq_setpoint1 = iq_s_1;
}
void ODrive_USB::set_vel_to_odrive(){
	float vel0 = ref_vel0;
	ep->setData(param_m0_input_vel, vel0);
	float vel1 = ref_vel1;
	ep->setData(param_m1_input_vel, vel1);
}

void ODrive_USB::get_endpoint_state_from_odrive(){
	bool ep0, ep1; 
	ep->getData(param_m0_endstop_state, ep0);
	ep->getData(param_m1_endstop_state, ep1);
	
	endpoint_state0 = ep0;
	endpoint_state1 = ep1;
}

void ODrive_USB::do_start_calibration(){
	start_calibration_cmd = true;
}

void ODrive_USB::run() {
    // Wait for drive to be initialized
	while(!started){
		usleep(10000);
	}
	
	int count = 0; // for watchdog feed
	
	// Motors calibration
	while(!start_calibration_cmd){
		get_endpoint_state_from_odrive(); // emergency button
		
// 		if(count % 500 == 0) {
// 			ep->execFunc(fcn_a0_watchdog_feed);
// 			ep->execFunc(fcn_a1_watchdog_feed);
// 			count = 0;
// 		}
// 		else 
// 			count++;
		
		usleep(10000);
	}
	
	calibrate_motors();
	
	// Send first feed and enable watchdog
	ep->execFunc(fcn_a0_watchdog_feed);
	ep->execFunc(fcn_a1_watchdog_feed);
	enable_watchdog();
	
	while(!calibration_done){
		get_endpoint_state_from_odrive(); // emergency button
		
		if(count % 500 == 0) {
			ep->execFunc(fcn_a0_watchdog_feed);
			ep->execFunc(fcn_a1_watchdog_feed);
			count = 0;
		}
		else 
			count++;
		
		usleep(10000);
	}
	
	// Start while loop
    running = true;
    while (running) {
// 		pc.tick();
	
		set_vel_to_odrive();
        get_vel_from_odrive();
		get_endpoint_state_from_odrive(); // emergency button
// 		get_current_from_odrive();        // TEST
		
		if(count % 500 == 0) {
			ep->execFunc(fcn_a0_watchdog_feed);
			ep->execFunc(fcn_a1_watchdog_feed);
			count = 0;
		}
		else 
			count++;
		
		usleep(1000);
// 				
// 		pc.tock();
// 		
// 		log.warn() << "period " << pc.period.min << "; " << pc.period.mean << "; " << pc.period.max;
// 		log.warn() << "run " << pc.run.min << "; " << pc.run.mean << "; " << pc.run.max;
// 		log.warn() << "--------------------------------";
	}
}

double ODrive_USB::get_encoder_vel(int motor){
	if(motor == 0)
		return (encoder_vel0/turns_per_rad);
	else if(motor == 1)
		return (encoder_vel1/turns_per_rad);
	else {
		log.error() << "Wrong motor ID. Encoder speed not valid";
		return 0;
	}
}

double ODrive_USB::get_current_setpoint(int motor){
	if(motor == 0)
		return (iq_setpoint0);
	else if(motor == 1)
		return (iq_setpoint1);
	else {
		log.error() << "Wrong motor ID. Encoder speed not valid";
		return 0;
	}
}

double ODrive_USB::get_current_measured(int motor){
	if(motor == 0)
		return (iq_measured0);
	else if(motor == 1)
		return (iq_measured1);
	else {
		log.error() << "Wrong motor ID. Encoder speed not valid";
		return 0;
	}
}

void ODrive_USB::set_ref_vel(int motor, float speed_rad){
	float speed_counts = turns_per_rad * speed_rad;
	
	if(motor == 0)      ref_vel0 = speed_counts;
	else if(motor == 1) ref_vel1 = speed_counts;
}

bool ODrive_USB::is_endstop_active(){	
	return (endpoint_state0 || endpoint_state1);
}

int ODrive_USB::getJson() {
    commBuffer rx, tx;
    int len, address = 0;
    std::string str;
    log.error() << "reading target configuration";
    do {
        ep->endpointRequest(0, rx, len, tx, true, 512, true, address);
        address = address + len;
        str.append((const char *)&rx[0], (size_t)len);
    } while (len > 0);
    Json::Reader reader;
    bool res = reader.parse(str, json);
    if (!res) {
        log.error() << "Error parsing json!";
        return 1;
    }
    return 0;
}
