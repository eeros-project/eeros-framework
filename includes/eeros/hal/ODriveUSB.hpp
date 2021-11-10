#ifndef ORG_EEROS_HAL_ODRIVE_USB_HP
#define ORG_EEROS_HAL_ODRIVE_USB_HP

#include <atomic>
#include <jsoncpp/json/json.h>
#include <math.h>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <odrive/odriveEP.hpp>

using namespace eeros::logger;

namespace eeros {
namespace hal {

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CTRL_MODE_VELOCITY_CONTROL 2
#define CTRL_MODE_POSITION_CONTROL 3 
  
  
/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::ODriveUSBInput class. 
 * Do not use it directly.
 *
 * @since v1.3
 */
class ODriveUSB : public eeros::Thread {
 public: 
  /**
   * Constructs a thread to communicate with ODrive device over USB \n
   *
   * @param serialNr - device serial number
   * @param encTicks - encoder ticks of motors connected
   * @param priority - execution priority of this thread
   * @param firstDrive - one drive must be set as the first one for reset tasks 
   */
  explicit ODriveUSB(uint64_t serialNr, float encTicks, int priority, bool firstDrive)
      : Thread(priority), serialNr(serialNr), encTicks(encTicks), firstDrive(firstDrive), 
        starting(true), running(false), log(Logger::getLogger('P')) {
    libusbContext = NULL;
    ep = new odrive::ODriveEP();
    int res = init(serialNr);
    if (res != 0) {
      log.error() << std::hex << "odrive with serial number 0x" << serialNr << " not found";
    }
    config();
    starting = false;
  }
      
  /**
   * Destructs the thread to get and send data from and to odrive \n
   */
  ~ODriveUSB() {
    // Set speed to zero
    ep->setData(param_m0_input_vel, (float)0.0); // motor0 vel_setpoint 
    ep->setData(param_m1_input_vel, (float)0.0); // motor1 vel_setpoint
    disableDrives();
    disable_watchdog();
    delete ep;
    if (libusbContext) { 
      libusb_exit(libusbContext); 
      libusbContext = NULL;
    }
    running = false;
  }

  
  int init(uint64_t odrive_serialnr){
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
    if(firstDrive) {
      for (__ssize_t i = 0; i < device_count; ++i) {
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
    for (__ssize_t i = 0; i < device_count; ++i) {
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

  int getJson() {
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
  
  double getEncoderVel(int motor, int turns_per_rev) {
    if (motor == 0) return encoder_vel0 / turns_per_rev;
    else if (motor == 1) return encoder_vel1 / turns_per_rev;
    else {
      log.error() << "Wrong motor ID. Encoder speed not valid";
      return 0;
    }
  }
  
  void setRefVel(int motor, float speed_rad, int turns_per_rev) {
    float speed_counts = turns_per_rev * speed_rad;
    if (motor == 0) ref_vel0 = speed_counts;
    else if(motor == 1) ref_vel1 = speed_counts;
  }
  
  void enableDrives(){
    ep->setData(param_a0_requested_state, AXIS_STATE_CLOSED_LOOP_CONTROL);
    ep->setData(param_a1_requested_state, AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  
  void disableDrives(){
    ep->setData(param_a0_requested_state, AXIS_STATE_IDLE);
    ep->setData(param_a1_requested_state, AXIS_STATE_IDLE);
  }
  
  void startCalibration(){
    startCalibrationCmd = true;
  }
  
  bool isEndstopActive(){	
    return (endpoint_state0 || endpoint_state1);
  }
  
  /**
   * Checks for drive calibration status
   * 
   * @return true, if odrive is calibrated 
   */
  bool isCalibrated(){
    return calibrated; 
  }
  
  /**
   * Returns current setpoint of one motor.
   * 
   * @param motor - motor id (0 or 1)
   * @return current setpoint of motor
   */
  double getCurrentSetpoint(int motor){
    if (motor == 0) return iq_setpoint0;
    else if (motor == 1) return iq_setpoint1;
    else {
      log.error() << "Wrong motor ID. Current not valid";
      return 0;
    }
  }
  
  /**
   * Returns measured current of one motor.
   * 
   * @param motor - motor id (0 or 1)
   * @return current of motor
   */
  double getCurrentMeasured(int motor){
    if (motor == 0) return iq_measured0;
    else if (motor == 1) return iq_measured1;
    else {
      log.error() << "Wrong motor ID. Current not valid";
      return 0;
    }
  }
  
  odrive::ODriveEP* ep;
  Json::Value json;
  
 private:
  libusb_context* libusbContext;
  uint64_t serialNr;
  float encTicks;
  bool firstDrive = false;
  bool startCalibrationCmd = false;
  bool calibrated = false;
  int nof_motors;
  std::atomic<bool> starting, running;
  std::atomic<double> encoder_vel0, encoder_vel1;
  std::atomic<float> ref_vel0, ref_vel1;
  std::atomic<bool> endpoint_state0, endpoint_state1;
  std::atomic<double> iq_setpoint0, iq_setpoint1, iq_measured0, iq_measured1;
  Logger log;
  
  /**
   * Configures odrive: clears errors, sets watchdog, endpoints, control mode, gains  \n
   */
  void config() {
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
    // ep->execFunc(fcn_save_configuration); // TODO still not working
  }
  
  void calibrateMotors(){
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
  
    log.info()  << serialNr << " -> M0: inductance = " << inductance0 << ", resistance = " << resistance0 
          << ", calibrated? " << m0_calibrated << ", encoder ready? " << m0_is_encoder_ready;
    log.info()  << serialNr << " -> M1: inductance = " << inductance1 << ", resistance = " << resistance1 
          << ", calibrated? " << m1_calibrated << ", encoder ready? " << m1_is_encoder_ready;
    
    // Motors calibration
    if(isinf(inductance0) || isinf(resistance0) || !m0_calibrated) {
      log.info() << serialNr << " -> Performing M0 calibration ... ";
      ep->setData(param_a0_requested_state, AXIS_STATE_MOTOR_CALIBRATION);
      sleep(5);
    }
    if(isinf(inductance1) || isinf(resistance1) || !m1_calibrated) {
      log.info() << serialNr << " -> Performing M1 calibration ... ";
      ep->setData(param_a1_requested_state, AXIS_STATE_MOTOR_CALIBRATION);
      sleep(5);
    }
    ep->getData(param_a0_is_calibrated, m0_calibrated);
    ep->getData(param_a1_is_calibrated, m1_calibrated);
    log.info() << serialNr << " -> Motor calibrated: M0 -> " << m0_calibrated << "; M1 -> " << m1_calibrated;
    // Encoder calibration
    ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
    if(!m0_is_encoder_ready) {
      log.info() << serialNr << " -> Start M0 encoder offset calibration ...";
      ep->setData(param_a0_requested_state, AXIS_STATE_ENCODER_OFFSET_CALIBRATION); 
    }
    ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
    if(!m1_is_encoder_ready) {
      log.info() << serialNr << " -> Start M1 encoder offset calibration ...";
      ep->setData(param_a1_requested_state, AXIS_STATE_ENCODER_OFFSET_CALIBRATION); 
    }
    // Wait for calibration to finish
    ep->getData(param_a0_is_calibrated, m0_calibrated);
    ep->getData(param_a1_is_calibrated, m1_calibrated);
    while (!m0_calibrated || !m0_is_encoder_ready || !m1_calibrated || !m1_is_encoder_ready){
      ep->getData(param_a0_is_calibrated, m0_calibrated);
      ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
      ep->getData(param_a1_is_calibrated, m1_calibrated);
      ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
      sleep(1);
    }
    ep->getData(param_m0_enc_is_ready, m0_is_encoder_ready);
    ep->getData(param_m1_enc_is_ready, m1_is_encoder_ready);
    log.info() << serialNr << " -> Motor calibrated: M0 -> " << m0_is_encoder_ready << "; M1 -> " << m1_is_encoder_ready;
    // Save configuration TODO, not working
    // ep->execFunc(save_configuration);
    // log.info() << "Configuration saved";
    calibrated = true;
  }

  /**
   * Sets control mode to velocity  \n
   */
  void set_velocity_ctrl_mode(){
    ep->setData(param_m0_control_mode, CTRL_MODE_VELOCITY_CONTROL);
    ep->setData(param_m1_control_mode, CTRL_MODE_VELOCITY_CONTROL);
  }
  
  /**
   * Sets controller gains values  \n
   */
  void set_controller_gains(){
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
  
  /**
   * Enables watchdog signal \n
   */
  void enable_watchdog() {
    ep->setData(param_a0_enable_watchdog, 1);
    ep->setData(param_a1_enable_watchdog, 1);
  }

  /**
   * Disables watchdog signal \n
   */
  void disable_watchdog() {
    ep->setData(param_a0_enable_watchdog, 0);
    ep->setData(param_a1_enable_watchdog, 0);
  }
  
  /**
   * Returns velocity values of motors connected to odrive \n
   */
  void get_vel_from_odrive(){
    float vel0, vel1;
    ep->getData(param_m0_enc_vel_estimate, vel0); // encoder, vel_estimate
    ep->getData(param_m1_enc_vel_estimate, vel1); // encoder, vel_estimate
    encoder_vel0 = vel0;
    encoder_vel1 = vel1;
  }
  
  /**
   * Sets velocity setpoints of motors connected to odrive \n
   */
  void set_vel_to_odrive(){
    float vel0 = ref_vel0;
    ep->setData(param_m0_input_vel, vel0);
    float vel1 = ref_vel1;
    ep->setData(param_m1_input_vel, vel1);
  }
  
  /**
   * Returns endpoint state from odrive \n
   */
  void get_endpoint_state_from_odrive(){
    bool ep0, ep1; 
    ep->getData(param_m0_endstop_state, ep0);
    ep->getData(param_m1_endstop_state, ep1);
    endpoint_state0 = ep0;
    endpoint_state1 = ep1;
  }
  
  /**
   * Returns current value from odrive \n
   */
  void get_current_from_odrive(){
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
  
  /**
   * Main run method. Gets measurements and sends setpoints \n
   */
  virtual void run(){
    while(starting) usleep(10000);
    int count = 0; // for watchdog feed
    
    while(!startCalibrationCmd){
      get_endpoint_state_from_odrive(); // emergency button
      usleep(10000);
    }
    calibrateMotors();
     // Send first feed and enable watchdog
    ep->execFunc(fcn_a0_watchdog_feed);
    ep->execFunc(fcn_a1_watchdog_feed);
    enable_watchdog();
    
    while(!calibrated){
      get_endpoint_state_from_odrive(); // emergency button
      
      if (count % 500 == 0) {
        ep->execFunc(fcn_a0_watchdog_feed);
        ep->execFunc(fcn_a1_watchdog_feed);
        count = 0;
      } else count++;
      usleep(10000);
    }
    running = true;
    while (running) {
      set_vel_to_odrive();
      get_vel_from_odrive();
      get_endpoint_state_from_odrive(); // emergency button
      if (count % 500 == 0) {
        ep->execFunc(fcn_a0_watchdog_feed);
        ep->execFunc(fcn_a1_watchdog_feed);
        count = 0;
      } else count++;
      usleep(1000);
    }
  }
  
  // ODrive settings
  static constexpr int param_odrv_enable_ascii_protocol_on_usb = 42; 
  static constexpr int param_odrv_brake_resistance = 44;
  static constexpr int param_odrv_dc_bus_undervoltage_trip_level = 45;
  static constexpr int param_odrv_serial_number = 4;
  static constexpr int param_odrv_uptime = 14;
  
  // Functions
  static constexpr int fcn_save_configuration = 544;
  static constexpr int fcn_a0_watchdog_feed = 298;
  static constexpr int fcn_a1_watchdog_feed = 527;
  static constexpr int fcn_m0_clear_errors = 299;
  static constexpr int fcn_m1_clear_errors = 528;
  
  // Axis 0
  static constexpr int param_a0_error = 71;
  static constexpr int param_a0_current_state = 73;
  static constexpr int param_a0_requested_state = 74;
  static constexpr int param_a0_loop_counter = 75;
  static constexpr int param_a0_is_homed = 77;
  static constexpr int param_a0_startup_motor_calibration = 78;
  static constexpr int param_a0_startup_encoder_index_search = 79;
  static constexpr int param_a0_startup_encoder_offset_calibration = 80;
  static constexpr int param_a0_startup_closed_loop_control = 81;
  static constexpr int param_a0_startup_sensorless_control = 82;
  static constexpr int param_a0_startup_homing = 83;
  static constexpr int param_a0_turns_per_step = 86; 
  static constexpr int param_a0_watchdog_timeout = 87;
  static constexpr int param_a0_enable_watchdog = 88;
  static constexpr int param_a0_is_calibrated = 134;
  
  // current control 
  static constexpr int param_m0_p_gain = 141;
  static constexpr int param_m0_i_gain = 142;
  static constexpr int param_m0_v_current_control_integral_d = 143;
  static constexpr int param_m0_v_current_control_integral_q = 144;
  
  // motor
  static constexpr int param_m0_iq_setpoint = 149;
  static constexpr int param_m0_iq_measured = 150;
// 	static constexpr int param_m0_pre_calibrated = 171;
  static constexpr int param_m0_pole_pairs = 172;
  static constexpr int param_m0_phase_inductance = 175;
  static constexpr int param_m0_phase_resistance = 176;
  static constexpr int param_m0_torque_constant = 177;
  static constexpr int param_m0_direction = 178;
  static constexpr int param_m0_current_lim = 180;
  static constexpr int param_m0_current_lim_margin = 181;
  static constexpr int param_m0_torque_lim = 182;
  
  // controller
  static constexpr int param_m0_controller_error = 193;
  static constexpr int param_m0_input_pos = 194;
  static constexpr int param_m0_input_vel = 195;
  static constexpr int param_m0_input_torque = 196;
  static constexpr int param_m0_pos_setpoint = 197;
  static constexpr int param_m0_vel_setpoint = 198;
  static constexpr int param_m0_torque_setpoint = 199;
  static constexpr int param_m0_trajectory_done = 200;
  static constexpr int param_m0_enable_vel_limit = 204;
  static constexpr int param_m0_enable_current_mode_vel_limit = 205;
  static constexpr int param_m0_control_mode = 208;
  static constexpr int param_m0_input_mode = 209;
  static constexpr int param_m0_pos_gain = 210;
  static constexpr int param_m0_vel_gain = 211;
  static constexpr int param_m0_vel_integrator_gain = 212;
  static constexpr int param_m0_vel_limit = 213;
  static constexpr int param_m0_homing_speed = 219;
  static constexpr int param_m0_inertia = 220;
  
  // encoder
  static constexpr int param_m0_enc_is_ready = 237;
  static constexpr int param_m0_enc_index_found = 238;
  static constexpr int param_m0_enc_count_in_cpr = 240;
  static constexpr int param_m0_enc_pos_estimate = 243;
  static constexpr int param_m0_enc_pos_estimate_counts = 244;
  static constexpr int param_m0_enc_vel_estimate = 249;
  static constexpr int param_m0_enc_vel_estimate_counts = 250;
  static constexpr int param_m0_enc_cpr = 259;
  static constexpr int param_m0_enc_pre_calibrated = 261;
  
  // gpio
  static constexpr int param_m0_endstop_state = 291;
  static constexpr int param_m0_endstop_gpio_num = 292;
  static constexpr int param_m0_endstop_enabled = 293;
  static constexpr int param_m0_endstop_is_active_high = 295;
  
  // Axis 1
  static constexpr int param_a1_error = 300;
  static constexpr int param_a1_current_state = 302;
  static constexpr int param_a1_requested_state = 303;
  static constexpr int param_a1_loop_counter = 304;
  static constexpr int param_a1_is_homed = 3006;
  static constexpr int param_a1_startup_motor_calibration = 307;
  static constexpr int param_a1_startup_encoder_index_search = 308;
  static constexpr int param_a1_startup_encoder_offset_calibration = 309;
  static constexpr int param_a1_startup_closed_loop_control = 310;
  static constexpr int param_a1_startup_sensorless_control = 311;
  static constexpr int param_a1_startup_homing = 312;
  static constexpr int param_a1_turns_per_step = 315; 
  static constexpr int param_a1_watchdog_timeout = 316;
  static constexpr int param_a1_enable_watchdog = 317;
  static constexpr int param_a1_is_calibrated = 363;

  // current control 
  static constexpr int param_m1_p_gain = 370;
  static constexpr int param_m1_i_gain = 371;
  static constexpr int param_m1_v_current_control_integral_d = 372;
  static constexpr int param_m1_v_current_control_integral_q = 373;
  
  // motor
  static constexpr int param_m1_iq_setpoint = 378;
  static constexpr int param_m1_iq_measured = 379;
// 	static constexpr int param_m1_pre_calibrated = 400;
  static constexpr int param_m1_pole_pairs = 401;
  static constexpr int param_m1_phase_inductance = 404;
  static constexpr int param_m1_phase_resistance = 405;
  static constexpr int param_m1_torque_constant = 406;
  static constexpr int param_m1_direction = 407;
  static constexpr int param_m1_current_lim = 409;
  static constexpr int param_m1_current_lim_margin = 410;
  static constexpr int param_m1_torque_lim = 411;
  
  // controller
  static constexpr int param_m1_controller_error = 422;
  static constexpr int param_m1_input_pos = 423;
  static constexpr int param_m1_input_vel = 424;
  static constexpr int param_m1_input_torque = 425;
  static constexpr int param_m1_pos_setpoint = 426;
  static constexpr int param_m1_vel_setpoint = 427;
  static constexpr int param_m1_torque_setpoint = 428;
  static constexpr int param_m1_trajectory_done = 429;
  static constexpr int param_m1_enable_vel_limit = 433;
  static constexpr int param_m1_enable_current_mode_vel_limit = 434;
  static constexpr int param_m1_control_mode = 437;
  static constexpr int param_m1_input_mode = 438;
  static constexpr int param_m1_pos_gain = 439;
  static constexpr int param_m1_vel_gain = 440;
  static constexpr int param_m1_vel_integrator_gain = 441;
  static constexpr int param_m1_vel_limit = 442;
  static constexpr int param_m1_homing_speed = 448;
  static constexpr int param_m1_inertia = 449;
  
  // encoder
  static constexpr int param_m1_enc_is_ready = 466;
  static constexpr int param_m1_enc_index_found = 467;
  static constexpr int param_m1_enc_count_in_cpr = 469;
  static constexpr int param_m1_enc_pos_estimate = 472;
  static constexpr int param_m1_enc_pos_estimate_counts = 473;
  static constexpr int param_m1_enc_vel_estimate = 478;
  static constexpr int param_m1_enc_vel_estimate_counts = 479;
  static constexpr int param_m1_enc_cpr = 488;
  static constexpr int param_m1_enc_pre_calibrated = 490;
  
  // gpio
  static constexpr int param_m1_endstop_state = 513;
  static constexpr int param_m1_endstop_gpio_num = 521;
  static constexpr int param_m1_endstop_enabled = 522;
  static constexpr int param_m1_endstop_is_active_high = 524;
};

}
}

#endif /* ORG_EEROS_HAL_ODRIVE_USB_HP */
