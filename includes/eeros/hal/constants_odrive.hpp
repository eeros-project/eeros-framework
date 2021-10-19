#ifndef ORG_EEROS_HAL_CONSTANTS_ODRIVE_HPP
#define ORG_EEROS_HAL_CONSTANTS_ODRIVE_HPP


namespace eeros {
namespace hal {
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
	
}
}

#endif /* ORG_EEROS_HAL_CONSTANTS_ODRIVE_HPP */
