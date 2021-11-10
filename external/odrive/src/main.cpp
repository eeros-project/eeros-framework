#include "odrive/odrive.hpp"
#include "../../../constants_odrive.hpp"
#include <iostream>

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_STARTUP_SEQUENCE 2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3
#define AXIS_STATE_MOTOR_CALIBRATION 4
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

using namespace odrive;

uint64_t odrv0_serialnr = 0x208c397d4d4d;

/**
* Test program for one odrive
* use endpoint id numbers given by the target configuration
* uncomment reading the json file for these id's
* all available odrive tool commands together with the associated id's are listed in include/odrive.hpp
* 
* Urs Graf, 28.Aug.2020
*/

int main(int argc, const char * argv[]) {
	uint64_t serialNumbers[1] = {odrv0_serialnr};
	float encoderTicksPerRad = 57.2958 * (2048 * 4) / 360.0;
	ODrive od (encoderTicksPerRad);

	int res = od.init(serialNumbers[0]);
	if (res != 0) {
		std::cerr << std::hex << "odrive with serial number 0x" << serialNumbers[0] << " not found" << std::endl;
		return EXIT_FAILURE;
	}

// // 	  // Read JSON from target
// 	  if (od.getJson()) {
// 	    return 1;
// 	  }
// 	  std::cout << od.json.toStyledString() << std::endl;

// 	od.ep->execFunc(387);
	std::cout << "init successful" << std::endl;
	
	uint64_t sn;
	od.ep->getData(param_odrv_serial_number, sn);
	std::cout << "Serial number ID " << param_odrv_serial_number << std::endl;
	uint32_t uptime_odrv;
	od.ep->getData(param_odrv_uptime, uptime_odrv);
	std::cout << std::dec << "Uptime: " << uptime_odrv << std::endl;
	
	// Calibration
	std::cout << "Calibration process started" << std::endl;
		
	// Run motor calibration and encoder offset only if 
	// Motor was not calibrated before
	bool is_pre_calibrated;
	od.ep->getData(param_a0_is_calibrated, is_pre_calibrated);
	std::cout << "is_pre_calibrated = " << is_pre_calibrated << std::endl;
	
	bool encoder_isReady;
	od.ep->getData(param_m0_enc_is_ready, encoder_isReady);
	std::cout << "encoder_isReady = " << encoder_isReady << std::endl;
	
	if(!is_pre_calibrated){
		std::cout << "Start full calibration: motor and encoder" << std::endl;
		od.ep->setData(param_a0_requested_state, AXIS_STATE_FULL_CALIBRATION_SEQUENCE); // axis0.requested_state
	}
	else if(!encoder_isReady) {
		std::cout << "Start only encoder offset calibration" << std::endl;
		od.ep->setData(param_a0_requested_state, AXIS_STATE_ENCODER_OFFSET_CALIBRATION); 
	}
	
	// Wait for calibration to finish
	bool motor_isCalibrated;
	od.ep->getData(param_a0_is_calibrated, motor_isCalibrated);
	od.ep->getData(param_m0_enc_is_ready, encoder_isReady);
	
	while (!motor_isCalibrated || !encoder_isReady){
		od.ep->getData(param_a0_is_calibrated, motor_isCalibrated);
		od.ep->getData(param_m0_enc_is_ready, encoder_isReady);
		std::cout << motor_isCalibrated << ", " << encoder_isReady << std::endl;
		sleep(1);
	}
	
// 	// Alternatively, specific startup sequence
// 	od.ep->setData(58, true); // axis0.config.startup_motor_calibration
// 	od.ep->setData(59, true); // axis0.startup_encoder_index_search
// 	od.ep->setData(60, true); // axis0.config.startup_encoder_offset_calibration
// 	od.ep->setData(61, true); // axis0.config.startup_closed_loop_control
// 	od.ep->setData(62, true); // axis0.config.startup_sensorless_control
// 	od.ep->setData(m0_requested_state, AXIS_STATE_STARTUP_SEQUENCE); // axis0.requested_state
	
	// Save configuration only if motor was calibrated
	od.ep->getData(125, is_pre_calibrated);
	if(!is_pre_calibrated) {
		std::cout << "Measure motor inductance and resistance" << std::endl;
		od.ep->setData(param_a0_requested_state, AXIS_STATE_MOTOR_CALIBRATION);
		
		float m0_inductance, m0_resistance;
		od.ep->getData(param_m0_phase_inductance, m0_inductance);
		od.ep->getData(param_m0_phase_resistance, m0_resistance);
		std::cout << "Motor inductance and resistance: " << m0_inductance << ", " << m0_resistance << std::endl;
		
		std::cout << "Set pre_calibrated = true" << std::endl;
		od.ep->setData(param_a0_is_calibrated, true); // axis0.motor.config.pre_calibrated
		sleep(5);
		std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
		std::cout << "Save_configuration" << std::endl;
		od.ep->execFunc(fcn_save_configuration);      // save_configuration()
	}
	sleep(15);
	// Set control mode and closed loop control
	od.ep->setData(param_m0_control_mode, 2);  // controller.config.control_mode (velocity)
	od.ep->setData(param_a0_requested_state, AXIS_STATE_CLOSED_LOOP_CONTROL);
			
	// Set speed, get positions
	int count = 0;
	float pos1, pos2;
	float velSet1 = 4000;
	uint8_t current_state;
	while (true) {
		count+=2;
		od.ep->getData(param_m0_enc_pos_estimate, pos1);  // encoder0 pos estimate
		od.ep->setData(param_m0_vel_setpoint, velSet1); // m0_vel_setpoint
		od.ep->getData(param_m1_enc_pos_estimate, pos2); // encoder1 pos estimate
		
		// Output data
		if (count % 1000 == 0)
			std::cout << "pos:" << pos1 << std::endl;
	
		if(count >= 6000 && count < 12000){
			velSet1 = -4000;
			od.ep->setData(param_m0_vel_setpoint, velSet1);
		}
		else if(count == 12000) {
			count = 0;
		}
		else {
			velSet1 = 4000;
			od.ep->setData(param_m0_vel_setpoint, velSet1);
		}
	}
	
	// Set speed to zero TODO - funktioniert nicht
	velSet1 = 0;
	od.ep->setData(param_m0_vel_setpoint, velSet1);
	
	// Disable motors
	od.ep->setData(param_a0_requested_state, AXIS_STATE_IDLE); // axis0.requested_state
}
