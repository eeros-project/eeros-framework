#ifndef ODRIVE_ENDPOINT_HPP_
#define ODRIVE_ENDPOINT_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <endian.h>
#include <mutex>
#include <libusb-1.0/libusb.h>

// ODrive Device Info
#define ODRIVE_USB_VENDORID     0x1209
#define ODRIVE_USB_PRODUCTID    0x0D32

// ODrive USB Protool
#define ODRIVE_TIMEOUT 200
#define ODRIVE_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_MAX_RESULT_LENGTH 100
#define ODRIVE_DEFAULT_CRC_VALUE 0x9b40
#define ODRIVE_PROTOCOL_VERION 1

// ODrive Comm
#define ODRIVE_COMM_SUCCESS 0
#define ODRIVE_COMM_ERROR   1

// Endpoints (from target)
#define CDC_IN_EP                                   0x81  /* EP1 for data IN (target) */
#define CDC_OUT_EP                                  0x01  /* EP1 for data OUT (target) */
#define CDC_CMD_EP                                  0x82  /* EP2 for CDC commands */
#define ODRIVE_IN_EP                                0x83  /* EP3 IN: ODrive device TX endpoint */
#define ODRIVE_OUT_EP                               0x03  /* EP3 OUT: ODrive device RX endpoint */

// CDC Endpoints parameters
#define CDC_DATA_HS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 

#define USB_CDC_CONFIG_DESC_SIZ                     (67 + 39)
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE
 
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23


typedef std::vector<uint8_t> commBuffer;

namespace odrive {
  
class ODriveEP {
 public:
  ODriveEP();
  ~ODriveEP();
  template<typename T> int getData(int id, T& value);
  template<typename T> int setData(int id, const T& value);
  int execFunc(int id);
  int endpointRequest(int endpoint_id, commBuffer& received_payload, int& received_length, 
                      commBuffer payload, bool ack, int length, bool read = false, int address = 0);
  libusb_device_handle *devHandle;
 private:
    short outboundSeqNo = 0; // unique ids for packets send to odrive
  std::mutex ep_lock;
  void appendShortToCommBuffer(commBuffer& buf, const short value);
  void appendIntToCommBuffer(commBuffer& buf, const int value);
  commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no);
  commBuffer createODrivePacket(short seq_no, int endpoint_id, short response_size, bool read, int address, const commBuffer& input);
};

}
#endif // ODRIVE_ENDPOINT_HPP_
 
//    reading target configuration
// [
//    {
//       "access" : "r",
//       "id" : 0,
//       "name" : "",
//       "type" : "json"
//    },
//    {
//       "access" : "r",
//       "id" : 1,
//       "name" : "vbus_voltage",
//       "type" : "float"
//    },
//    {
//       "access" : "r",
//       "id" : 2,
//       "name" : "ibus",
//       "type" : "float"
//    },
//    {
//       "access" : "rw",
//       "id" : 3,
//       "name" : "ibus_report_filter_k",
//       "type" : "float"
//    },
//    {
//       "access" : "r",
//       "id" : 4,
//       "name" : "serial_number",
//       "type" : "uint64"
//    },
//    {
//       "access" : "r",
//       "id" : 5,
//       "name" : "hw_version_major",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 6,
//       "name" : "hw_version_minor",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 7,
//       "name" : "hw_version_variant",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 8,
//       "name" : "fw_version_major",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 9,
//       "name" : "fw_version_minor",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 10,
//       "name" : "fw_version_revision",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 11,
//       "name" : "fw_version_unreleased",
//       "type" : "uint8"
//    },
//    {
//       "access" : "r",
//       "id" : 12,
//       "name" : "brake_resistor_armed",
//       "type" : "bool"
//    },
//    {
//       "access" : "rw",
//       "id" : 13,
//       "name" : "brake_resistor_saturated",
//       "type" : "bool"
//    },
//    {
//       "members" : [
//          {
//             "access" : "r",
//             "id" : 14,
//             "name" : "uptime",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 15,
//             "name" : "min_heap_space",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 16,
//             "name" : "min_stack_space_axis0",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 17,
//             "name" : "min_stack_space_axis1",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 18,
//             "name" : "min_stack_space_comms",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 19,
//             "name" : "min_stack_space_usb",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 20,
//             "name" : "min_stack_space_uart",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 21,
//             "name" : "min_stack_space_can",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 22,
//             "name" : "min_stack_space_usb_irq",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 23,
//             "name" : "min_stack_space_startup",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 24,
//             "name" : "stack_usage_axis0",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 25,
//             "name" : "stack_usage_axis1",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 26,
//             "name" : "stack_usage_comms",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 27,
//             "name" : "stack_usage_usb",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 28,
//             "name" : "stack_usage_uart",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 29,
//             "name" : "stack_usage_usb_irq",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 30,
//             "name" : "stack_usage_startup",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 31,
//             "name" : "stack_usage_can",
//             "type" : "uint32"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 32,
//                   "name" : "rx_cnt",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 33,
//                   "name" : "tx_cnt",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 34,
//                   "name" : "tx_overrun_cnt",
//                   "type" : "uint32"
//                }
//             ],
//             "name" : "usb",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 35,
//                   "name" : "addr",
//                   "type" : "uint8"
//                },
//                {
//                   "access" : "r",
//                   "id" : 36,
//                   "name" : "addr_match_cnt",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 37,
//                   "name" : "rx_cnt",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 38,
//                   "name" : "error_cnt",
//                   "type" : "uint32"
//                }
//             ],
//             "name" : "i2c",
//             "type" : "object"
//          }
//       ],
//       "name" : "system_stats",
//       "type" : "object"
//    },
//    {
//       "members" : [
//          {
//             "access" : "rw",
//             "id" : 39,
//             "name" : "enable_uart",
//             "type" : "bool"
//          },
//          {
//             "access" : "rw",
//             "id" : 40,
//             "name" : "uart_baudrate",
//             "type" : "uint32"
//          },
//          {
//             "access" : "rw",
//             "id" : 41,
//             "name" : "enable_i2c_instead_of_can",
//             "type" : "bool"
//          },
//          {
//             "access" : "rw",
//             "id" : 42,
//             "name" : "enable_ascii_protocol_on_usb",
//             "type" : "bool"
//          },
//          {
//             "access" : "rw",
//             "id" : 43,
//             "name" : "max_regen_current",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 44,
//             "name" : "brake_resistance",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 45,
//             "name" : "dc_bus_undervoltage_trip_level",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 46,
//             "name" : "dc_bus_overvoltage_trip_level",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 47,
//             "name" : "enable_dc_bus_overvoltage_ramp",
//             "type" : "bool"
//          },
//          {
//             "access" : "rw",
//             "id" : 48,
//             "name" : "dc_bus_overvoltage_ramp_start",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 49,
//             "name" : "dc_bus_overvoltage_ramp_end",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 50,
//             "name" : "dc_max_positive_current",
//             "type" : "float"
//          },
//          {
//             "access" : "rw",
//             "id" : 51,
//             "name" : "dc_max_negative_current",
//             "type" : "float"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 52,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 53,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 54,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio1_pwm_mapping",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 55,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 56,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 57,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio2_pwm_mapping",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 58,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 59,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 60,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio3_pwm_mapping",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 61,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 62,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 63,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio4_pwm_mapping",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 64,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 65,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 66,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio3_analog_mapping",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 67,
//                   "name" : "endpoint",
//                   "type" : "endpoint_ref"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 68,
//                   "name" : "min",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 69,
//                   "name" : "max",
//                   "type" : "float"
//                }
//             ],
//             "name" : "gpio4_analog_mapping",
//             "type" : "object"
//          }
//       ],
//       "name" : "config",
//       "type" : "object"
//    },
//    {
//       "access" : "r",
//       "id" : 70,
//       "name" : "user_config_loaded",
//       "type" : "bool"
//    },
//    {
//       "members" : [
//          {
//             "access" : "rw",
//             "id" : 71,
//             "name" : "error",
//             "type" : "int32"
//          },
//          {
//             "access" : "r",
//             "id" : 72,
//             "name" : "step_dir_active",
//             "type" : "bool"
//          },
//          {
//             "access" : "r",
//             "id" : 73,
//             "name" : "current_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "rw",
//             "id" : 74,
//             "name" : "requested_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "r",
//             "id" : 75,
//             "name" : "loop_counter",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 76,
//             "name" : "lockin_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "rw",
//             "id" : 77,
//             "name" : "is_homed",
//             "type" : "bool"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 78,
//                   "name" : "startup_motor_calibration",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 79,
//                   "name" : "startup_encoder_index_search",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 80,
//                   "name" : "startup_encoder_offset_calibration",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 81,
//                   "name" : "startup_closed_loop_control",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 82,
//                   "name" : "startup_sensorless_control",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 83,
//                   "name" : "startup_homing",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 84,
//                   "name" : "enable_step_dir",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 85,
//                   "name" : "step_dir_always_on",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 86,
//                   "name" : "turns_per_step",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 87,
//                   "name" : "watchdog_timeout",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 88,
//                   "name" : "enable_watchdog",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 89,
//                   "name" : "step_gpio_pin",
//                   "type" : "uint16"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 90,
//                   "name" : "dir_gpio_pin",
//                   "type" : "uint16"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 91,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 92,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 93,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 94,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 95,
//                         "name" : "vel",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "calibration_lockin",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 96,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 97,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 98,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 99,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 100,
//                         "name" : "vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 101,
//                         "name" : "finish_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 102,
//                         "name" : "finish_on_vel",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 103,
//                         "name" : "finish_on_distance",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 104,
//                         "name" : "finish_on_enc_idx",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "sensorless_ramp",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 105,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 106,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 107,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 108,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 109,
//                         "name" : "vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 110,
//                         "name" : "finish_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 111,
//                         "name" : "finish_on_vel",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 112,
//                         "name" : "finish_on_distance",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 113,
//                         "name" : "finish_on_enc_idx",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "general_lockin",
//                   "type" : "object"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 114,
//                   "name" : "can_node_id",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 115,
//                   "name" : "can_node_id_extended",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 116,
//                   "name" : "can_heartbeat_rate_ms",
//                   "type" : "uint32"
//                }
//             ],
//             "name" : "config",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 117,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 118,
//                   "name" : "temperature",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 119,
//                         "name" : "temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 120,
//                         "name" : "temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 121,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "fet_thermistor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 122,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 123,
//                   "name" : "temperature",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 124,
//                         "name" : "gpio_pin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 125,
//                         "name" : "poly_coefficient_0",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 126,
//                         "name" : "poly_coefficient_1",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 127,
//                         "name" : "poly_coefficient_2",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 128,
//                         "name" : "poly_coefficient_3",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 129,
//                         "name" : "temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 130,
//                         "name" : "temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 131,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "motor_thermistor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 132,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 133,
//                   "name" : "armed_state",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 134,
//                   "name" : "is_calibrated",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 135,
//                   "name" : "current_meas_phB",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 136,
//                   "name" : "current_meas_phC",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 137,
//                   "name" : "DC_calib_phB",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 138,
//                   "name" : "DC_calib_phC",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 139,
//                   "name" : "phase_current_rev_gain",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 140,
//                   "name" : "effective_current_lim",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 141,
//                         "name" : "p_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 142,
//                         "name" : "i_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 143,
//                         "name" : "v_current_control_integral_d",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 144,
//                         "name" : "v_current_control_integral_q",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 145,
//                         "name" : "Ibus",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 146,
//                         "name" : "final_v_alpha",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 147,
//                         "name" : "final_v_beta",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 148,
//                         "name" : "Id_setpoint",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 149,
//                         "name" : "Iq_setpoint",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 150,
//                         "name" : "Iq_measured",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 151,
//                         "name" : "Id_measured",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 152,
//                         "name" : "I_measured_report_filter_k",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 153,
//                         "name" : "max_allowed_current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 154,
//                         "name" : "overcurrent_trip_level",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 155,
//                         "name" : "acim_rotor_flux",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 156,
//                         "name" : "async_phase_vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 157,
//                         "name" : "async_phase_offset",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "current_control",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "r",
//                         "id" : 158,
//                         "name" : "drv_fault",
//                         "type" : "int32"
//                      }
//                   ],
//                   "name" : "gate_driver",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "r",
//                         "id" : 159,
//                         "name" : "general",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 160,
//                         "name" : "adc_cb_i",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 161,
//                         "name" : "adc_cb_dc",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 162,
//                         "name" : "meas_r",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 163,
//                         "name" : "meas_l",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 164,
//                         "name" : "enc_calib",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 165,
//                         "name" : "idx_search",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 166,
//                         "name" : "foc_voltage",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 167,
//                         "name" : "foc_current",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 168,
//                         "name" : "spi_start",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 169,
//                         "name" : "sample_now",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 170,
//                         "name" : "spi_end",
//                         "type" : "uint16"
//                      }
//                   ],
//                   "name" : "timing_log",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 171,
//                         "name" : "pre_calibrated",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 172,
//                         "name" : "pole_pairs",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 173,
//                         "name" : "calibration_current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 174,
//                         "name" : "resistance_calib_max_voltage",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 175,
//                         "name" : "phase_inductance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 176,
//                         "name" : "phase_resistance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 177,
//                         "name" : "torque_constant",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 178,
//                         "name" : "direction",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 179,
//                         "name" : "motor_type",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 180,
//                         "name" : "current_lim",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 181,
//                         "name" : "current_lim_margin",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 182,
//                         "name" : "torque_lim",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 183,
//                         "name" : "inverter_temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 184,
//                         "name" : "inverter_temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 185,
//                         "name" : "requested_current_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 186,
//                         "name" : "current_control_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 187,
//                         "name" : "acim_slip_velocity",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 188,
//                         "name" : "acim_gain_min_flux",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 189,
//                         "name" : "acim_autoflux_min_Id",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 190,
//                         "name" : "acim_autoflux_enable",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 191,
//                         "name" : "acim_autoflux_attack_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 192,
//                         "name" : "acim_autoflux_decay_gain",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "motor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 193,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 194,
//                   "name" : "input_pos",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 195,
//                   "name" : "input_vel",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 196,
//                   "name" : "input_torque",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 197,
//                   "name" : "pos_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 198,
//                   "name" : "vel_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 199,
//                   "name" : "torque_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 200,
//                   "name" : "trajectory_done",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 201,
//                   "name" : "vel_integrator_torque",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 202,
//                   "name" : "anticogging_valid",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 203,
//                         "name" : "gain_scheduling_width",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 204,
//                         "name" : "enable_vel_limit",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 205,
//                         "name" : "enable_current_mode_vel_limit",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 206,
//                         "name" : "enable_gain_scheduling",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 207,
//                         "name" : "enable_overspeed_error",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 208,
//                         "name" : "control_mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 209,
//                         "name" : "input_mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 210,
//                         "name" : "pos_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 211,
//                         "name" : "vel_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 212,
//                         "name" : "vel_integrator_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 213,
//                         "name" : "vel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 214,
//                         "name" : "vel_limit_tolerance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 215,
//                         "name" : "vel_ramp_rate",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 216,
//                         "name" : "torque_ramp_rate",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 217,
//                         "name" : "circular_setpoints",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 218,
//                         "name" : "circular_setpoint_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 219,
//                         "name" : "homing_speed",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 220,
//                         "name" : "inertia",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 221,
//                         "name" : "axis_to_mirror",
//                         "type" : "uint8"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 222,
//                         "name" : "mirror_ratio",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 223,
//                         "name" : "load_encoder_axis",
//                         "type" : "uint8"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 224,
//                         "name" : "input_filter_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "members" : [
//                            {
//                               "access" : "r",
//                               "id" : 225,
//                               "name" : "index",
//                               "type" : "uint32"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 226,
//                               "name" : "pre_calibrated",
//                               "type" : "bool"
//                            },
//                            {
//                               "access" : "r",
//                               "id" : 227,
//                               "name" : "calib_anticogging",
//                               "type" : "bool"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 228,
//                               "name" : "calib_pos_threshold",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 229,
//                               "name" : "calib_vel_threshold",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "r",
//                               "id" : 230,
//                               "name" : "cogging_ratio",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 231,
//                               "name" : "anticogging_enabled",
//                               "type" : "bool"
//                            }
//                         ],
//                         "name" : "anticogging",
//                         "type" : "object"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                },
//                {
//                   "id" : 232,
//                   "inputs" : [
//                      {
//                         "access" : "rw",
//                         "id" : 233,
//                         "name" : "displacement",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 234,
//                         "name" : "from_input_pos",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "move_incremental",
//                   "outputs" : [],
//                   "type" : "function"
//                },
//                {
//                   "id" : 235,
//                   "inputs" : [],
//                   "name" : "start_anticogging_calibration",
//                   "outputs" : [],
//                   "type" : "function"
//                }
//             ],
//             "name" : "controller",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 236,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 237,
//                   "name" : "is_ready",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 238,
//                   "name" : "index_found",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 239,
//                   "name" : "shadow_count",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 240,
//                   "name" : "count_in_cpr",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 241,
//                   "name" : "interpolation",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 242,
//                   "name" : "phase",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 243,
//                   "name" : "pos_estimate",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 244,
//                   "name" : "pos_estimate_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 245,
//                   "name" : "pos_cpr",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 246,
//                   "name" : "pos_cpr_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 247,
//                   "name" : "pos_circular",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 248,
//                   "name" : "hall_state",
//                   "type" : "uint8"
//                },
//                {
//                   "access" : "r",
//                   "id" : 249,
//                   "name" : "vel_estimate",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 250,
//                   "name" : "vel_estimate_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 251,
//                   "name" : "calib_scan_response",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 252,
//                   "name" : "pos_abs",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 253,
//                   "name" : "spi_error_rate",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 254,
//                         "name" : "mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 255,
//                         "name" : "use_index",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 256,
//                         "name" : "find_idx_on_lockin_only",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 257,
//                         "name" : "abs_spi_cs_gpio_pin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 258,
//                         "name" : "zero_count_on_find_idx",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 259,
//                         "name" : "cpr",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 260,
//                         "name" : "offset",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 261,
//                         "name" : "pre_calibrated",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 262,
//                         "name" : "offset_float",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 263,
//                         "name" : "enable_phase_interpolation",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 264,
//                         "name" : "bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 265,
//                         "name" : "calib_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 266,
//                         "name" : "calib_scan_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 267,
//                         "name" : "calib_scan_omega",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 268,
//                         "name" : "idx_search_unidirectional",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 269,
//                         "name" : "ignore_illegal_hall_state",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 270,
//                         "name" : "sincos_gpio_pin_sin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 271,
//                         "name" : "sincos_gpio_pin_cos",
//                         "type" : "uint16"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                },
//                {
//                   "id" : 272,
//                   "inputs" : [
//                      {
//                         "access" : "rw",
//                         "id" : 273,
//                         "name" : "count",
//                         "type" : "int32"
//                      }
//                   ],
//                   "name" : "set_linear_count",
//                   "outputs" : [],
//                   "type" : "function"
//                }
//             ],
//             "name" : "encoder",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 274,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 275,
//                   "name" : "phase",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 276,
//                   "name" : "pll_pos",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 277,
//                   "name" : "vel_estimate",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 278,
//                         "name" : "observer_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 279,
//                         "name" : "pll_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 280,
//                         "name" : "pm_flux_linkage",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "sensorless_estimator",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 281,
//                         "name" : "vel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 282,
//                         "name" : "accel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 283,
//                         "name" : "decel_limit",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "trap_traj",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 284,
//                   "name" : "endstop_state",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 285,
//                         "name" : "gpio_num",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 286,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 287,
//                         "name" : "offset",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 288,
//                         "name" : "is_active_high",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 289,
//                         "name" : "pullup",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 290,
//                         "name" : "debounce_ms",
//                         "type" : "uint32"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "min_endstop",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 291,
//                   "name" : "endstop_state",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 292,
//                         "name" : "gpio_num",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 293,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 294,
//                         "name" : "offset",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 295,
//                         "name" : "is_active_high",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 296,
//                         "name" : "pullup",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 297,
//                         "name" : "debounce_ms",
//                         "type" : "uint32"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "max_endstop",
//             "type" : "object"
//          },
//          {
//             "id" : 298,
//             "inputs" : [],
//             "name" : "watchdog_feed",
//             "outputs" : [],
//             "type" : "function"
//          },
//          {
//             "id" : 299,
//             "inputs" : [],
//             "name" : "clear_errors",
//             "outputs" : [],
//             "type" : "function"
//          }
//       ],
//       "name" : "axis0",
//       "type" : "object"
//    },
//    {
//       "members" : [
//          {
//             "access" : "rw",
//             "id" : 300,
//             "name" : "error",
//             "type" : "int32"
//          },
//          {
//             "access" : "r",
//             "id" : 301,
//             "name" : "step_dir_active",
//             "type" : "bool"
//          },
//          {
//             "access" : "r",
//             "id" : 302,
//             "name" : "current_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "rw",
//             "id" : 303,
//             "name" : "requested_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "r",
//             "id" : 304,
//             "name" : "loop_counter",
//             "type" : "uint32"
//          },
//          {
//             "access" : "r",
//             "id" : 305,
//             "name" : "lockin_state",
//             "type" : "int32"
//          },
//          {
//             "access" : "rw",
//             "id" : 306,
//             "name" : "is_homed",
//             "type" : "bool"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 307,
//                   "name" : "startup_motor_calibration",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 308,
//                   "name" : "startup_encoder_index_search",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 309,
//                   "name" : "startup_encoder_offset_calibration",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 310,
//                   "name" : "startup_closed_loop_control",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 311,
//                   "name" : "startup_sensorless_control",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 312,
//                   "name" : "startup_homing",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 313,
//                   "name" : "enable_step_dir",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 314,
//                   "name" : "step_dir_always_on",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 315,
//                   "name" : "turns_per_step",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 316,
//                   "name" : "watchdog_timeout",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 317,
//                   "name" : "enable_watchdog",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 318,
//                   "name" : "step_gpio_pin",
//                   "type" : "uint16"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 319,
//                   "name" : "dir_gpio_pin",
//                   "type" : "uint16"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 320,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 321,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 322,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 323,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 324,
//                         "name" : "vel",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "calibration_lockin",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 325,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 326,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 327,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 328,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 329,
//                         "name" : "vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 330,
//                         "name" : "finish_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 331,
//                         "name" : "finish_on_vel",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 332,
//                         "name" : "finish_on_distance",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 333,
//                         "name" : "finish_on_enc_idx",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "sensorless_ramp",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 334,
//                         "name" : "current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 335,
//                         "name" : "ramp_time",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 336,
//                         "name" : "ramp_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 337,
//                         "name" : "accel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 338,
//                         "name" : "vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 339,
//                         "name" : "finish_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 340,
//                         "name" : "finish_on_vel",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 341,
//                         "name" : "finish_on_distance",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 342,
//                         "name" : "finish_on_enc_idx",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "general_lockin",
//                   "type" : "object"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 343,
//                   "name" : "can_node_id",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 344,
//                   "name" : "can_node_id_extended",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 345,
//                   "name" : "can_heartbeat_rate_ms",
//                   "type" : "uint32"
//                }
//             ],
//             "name" : "config",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 346,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 347,
//                   "name" : "temperature",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 348,
//                         "name" : "temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 349,
//                         "name" : "temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 350,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "fet_thermistor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 351,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 352,
//                   "name" : "temperature",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 353,
//                         "name" : "gpio_pin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 354,
//                         "name" : "poly_coefficient_0",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 355,
//                         "name" : "poly_coefficient_1",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 356,
//                         "name" : "poly_coefficient_2",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 357,
//                         "name" : "poly_coefficient_3",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 358,
//                         "name" : "temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 359,
//                         "name" : "temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 360,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "motor_thermistor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 361,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 362,
//                   "name" : "armed_state",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 363,
//                   "name" : "is_calibrated",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 364,
//                   "name" : "current_meas_phB",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 365,
//                   "name" : "current_meas_phC",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 366,
//                   "name" : "DC_calib_phB",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 367,
//                   "name" : "DC_calib_phC",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 368,
//                   "name" : "phase_current_rev_gain",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 369,
//                   "name" : "effective_current_lim",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 370,
//                         "name" : "p_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 371,
//                         "name" : "i_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 372,
//                         "name" : "v_current_control_integral_d",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 373,
//                         "name" : "v_current_control_integral_q",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 374,
//                         "name" : "Ibus",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 375,
//                         "name" : "final_v_alpha",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 376,
//                         "name" : "final_v_beta",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 377,
//                         "name" : "Id_setpoint",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 378,
//                         "name" : "Iq_setpoint",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 379,
//                         "name" : "Iq_measured",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 380,
//                         "name" : "Id_measured",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 381,
//                         "name" : "I_measured_report_filter_k",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 382,
//                         "name" : "max_allowed_current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 383,
//                         "name" : "overcurrent_trip_level",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 384,
//                         "name" : "acim_rotor_flux",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 385,
//                         "name" : "async_phase_vel",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 386,
//                         "name" : "async_phase_offset",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "current_control",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "r",
//                         "id" : 387,
//                         "name" : "drv_fault",
//                         "type" : "int32"
//                      }
//                   ],
//                   "name" : "gate_driver",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "r",
//                         "id" : 388,
//                         "name" : "general",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 389,
//                         "name" : "adc_cb_i",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 390,
//                         "name" : "adc_cb_dc",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 391,
//                         "name" : "meas_r",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 392,
//                         "name" : "meas_l",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 393,
//                         "name" : "enc_calib",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 394,
//                         "name" : "idx_search",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 395,
//                         "name" : "foc_voltage",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 396,
//                         "name" : "foc_current",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 397,
//                         "name" : "spi_start",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 398,
//                         "name" : "sample_now",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "r",
//                         "id" : 399,
//                         "name" : "spi_end",
//                         "type" : "uint16"
//                      }
//                   ],
//                   "name" : "timing_log",
//                   "type" : "object"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 400,
//                         "name" : "pre_calibrated",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 401,
//                         "name" : "pole_pairs",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 402,
//                         "name" : "calibration_current",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 403,
//                         "name" : "resistance_calib_max_voltage",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 404,
//                         "name" : "phase_inductance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 405,
//                         "name" : "phase_resistance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 406,
//                         "name" : "torque_constant",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 407,
//                         "name" : "direction",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 408,
//                         "name" : "motor_type",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 409,
//                         "name" : "current_lim",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 410,
//                         "name" : "current_lim_margin",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 411,
//                         "name" : "torque_lim",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 412,
//                         "name" : "inverter_temp_limit_lower",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 413,
//                         "name" : "inverter_temp_limit_upper",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 414,
//                         "name" : "requested_current_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 415,
//                         "name" : "current_control_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 416,
//                         "name" : "acim_slip_velocity",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 417,
//                         "name" : "acim_gain_min_flux",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 418,
//                         "name" : "acim_autoflux_min_Id",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 419,
//                         "name" : "acim_autoflux_enable",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 420,
//                         "name" : "acim_autoflux_attack_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 421,
//                         "name" : "acim_autoflux_decay_gain",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "motor",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 422,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 423,
//                   "name" : "input_pos",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 424,
//                   "name" : "input_vel",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 425,
//                   "name" : "input_torque",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 426,
//                   "name" : "pos_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 427,
//                   "name" : "vel_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 428,
//                   "name" : "torque_setpoint",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 429,
//                   "name" : "trajectory_done",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 430,
//                   "name" : "vel_integrator_torque",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 431,
//                   "name" : "anticogging_valid",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 432,
//                         "name" : "gain_scheduling_width",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 433,
//                         "name" : "enable_vel_limit",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 434,
//                         "name" : "enable_current_mode_vel_limit",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 435,
//                         "name" : "enable_gain_scheduling",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 436,
//                         "name" : "enable_overspeed_error",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 437,
//                         "name" : "control_mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 438,
//                         "name" : "input_mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 439,
//                         "name" : "pos_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 440,
//                         "name" : "vel_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 441,
//                         "name" : "vel_integrator_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 442,
//                         "name" : "vel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 443,
//                         "name" : "vel_limit_tolerance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 444,
//                         "name" : "vel_ramp_rate",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 445,
//                         "name" : "torque_ramp_rate",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 446,
//                         "name" : "circular_setpoints",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 447,
//                         "name" : "circular_setpoint_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 448,
//                         "name" : "homing_speed",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 449,
//                         "name" : "inertia",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 450,
//                         "name" : "axis_to_mirror",
//                         "type" : "uint8"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 451,
//                         "name" : "mirror_ratio",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 452,
//                         "name" : "load_encoder_axis",
//                         "type" : "uint8"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 453,
//                         "name" : "input_filter_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "members" : [
//                            {
//                               "access" : "r",
//                               "id" : 454,
//                               "name" : "index",
//                               "type" : "uint32"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 455,
//                               "name" : "pre_calibrated",
//                               "type" : "bool"
//                            },
//                            {
//                               "access" : "r",
//                               "id" : 456,
//                               "name" : "calib_anticogging",
//                               "type" : "bool"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 457,
//                               "name" : "calib_pos_threshold",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 458,
//                               "name" : "calib_vel_threshold",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "r",
//                               "id" : 459,
//                               "name" : "cogging_ratio",
//                               "type" : "float"
//                            },
//                            {
//                               "access" : "rw",
//                               "id" : 460,
//                               "name" : "anticogging_enabled",
//                               "type" : "bool"
//                            }
//                         ],
//                         "name" : "anticogging",
//                         "type" : "object"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                },
//                {
//                   "id" : 461,
//                   "inputs" : [
//                      {
//                         "access" : "rw",
//                         "id" : 462,
//                         "name" : "displacement",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 463,
//                         "name" : "from_input_pos",
//                         "type" : "bool"
//                      }
//                   ],
//                   "name" : "move_incremental",
//                   "outputs" : [],
//                   "type" : "function"
//                },
//                {
//                   "id" : 464,
//                   "inputs" : [],
//                   "name" : "start_anticogging_calibration",
//                   "outputs" : [],
//                   "type" : "function"
//                }
//             ],
//             "name" : "controller",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 465,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 466,
//                   "name" : "is_ready",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 467,
//                   "name" : "index_found",
//                   "type" : "bool"
//                },
//                {
//                   "access" : "r",
//                   "id" : 468,
//                   "name" : "shadow_count",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 469,
//                   "name" : "count_in_cpr",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 470,
//                   "name" : "interpolation",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 471,
//                   "name" : "phase",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 472,
//                   "name" : "pos_estimate",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 473,
//                   "name" : "pos_estimate_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 474,
//                   "name" : "pos_cpr",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 475,
//                   "name" : "pos_cpr_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 476,
//                   "name" : "pos_circular",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 477,
//                   "name" : "hall_state",
//                   "type" : "uint8"
//                },
//                {
//                   "access" : "r",
//                   "id" : 478,
//                   "name" : "vel_estimate",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 479,
//                   "name" : "vel_estimate_counts",
//                   "type" : "float"
//                },
//                {
//                   "access" : "r",
//                   "id" : 480,
//                   "name" : "calib_scan_response",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 481,
//                   "name" : "pos_abs",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "r",
//                   "id" : 482,
//                   "name" : "spi_error_rate",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 483,
//                         "name" : "mode",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 484,
//                         "name" : "use_index",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 485,
//                         "name" : "find_idx_on_lockin_only",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 486,
//                         "name" : "abs_spi_cs_gpio_pin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 487,
//                         "name" : "zero_count_on_find_idx",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 488,
//                         "name" : "cpr",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 489,
//                         "name" : "offset",
//                         "type" : "int32"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 490,
//                         "name" : "pre_calibrated",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 491,
//                         "name" : "offset_float",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 492,
//                         "name" : "enable_phase_interpolation",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 493,
//                         "name" : "bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 494,
//                         "name" : "calib_range",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 495,
//                         "name" : "calib_scan_distance",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 496,
//                         "name" : "calib_scan_omega",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 497,
//                         "name" : "idx_search_unidirectional",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 498,
//                         "name" : "ignore_illegal_hall_state",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 499,
//                         "name" : "sincos_gpio_pin_sin",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 500,
//                         "name" : "sincos_gpio_pin_cos",
//                         "type" : "uint16"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                },
//                {
//                   "id" : 501,
//                   "inputs" : [
//                      {
//                         "access" : "rw",
//                         "id" : 502,
//                         "name" : "count",
//                         "type" : "int32"
//                      }
//                   ],
//                   "name" : "set_linear_count",
//                   "outputs" : [],
//                   "type" : "function"
//                }
//             ],
//             "name" : "encoder",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "rw",
//                   "id" : 503,
//                   "name" : "error",
//                   "type" : "int32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 504,
//                   "name" : "phase",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 505,
//                   "name" : "pll_pos",
//                   "type" : "float"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 506,
//                   "name" : "vel_estimate",
//                   "type" : "float"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 507,
//                         "name" : "observer_gain",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 508,
//                         "name" : "pll_bandwidth",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 509,
//                         "name" : "pm_flux_linkage",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "sensorless_estimator",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 510,
//                         "name" : "vel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 511,
//                         "name" : "accel_limit",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 512,
//                         "name" : "decel_limit",
//                         "type" : "float"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "trap_traj",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 513,
//                   "name" : "endstop_state",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 514,
//                         "name" : "gpio_num",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 515,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 516,
//                         "name" : "offset",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 517,
//                         "name" : "is_active_high",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 518,
//                         "name" : "pullup",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 519,
//                         "name" : "debounce_ms",
//                         "type" : "uint32"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "min_endstop",
//             "type" : "object"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 520,
//                   "name" : "endstop_state",
//                   "type" : "bool"
//                },
//                {
//                   "members" : [
//                      {
//                         "access" : "rw",
//                         "id" : 521,
//                         "name" : "gpio_num",
//                         "type" : "uint16"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 522,
//                         "name" : "enabled",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 523,
//                         "name" : "offset",
//                         "type" : "float"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 524,
//                         "name" : "is_active_high",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 525,
//                         "name" : "pullup",
//                         "type" : "bool"
//                      },
//                      {
//                         "access" : "rw",
//                         "id" : 526,
//                         "name" : "debounce_ms",
//                         "type" : "uint32"
//                      }
//                   ],
//                   "name" : "config",
//                   "type" : "object"
//                }
//             ],
//             "name" : "max_endstop",
//             "type" : "object"
//          },
//          {
//             "id" : 527,
//             "inputs" : [],
//             "name" : "watchdog_feed",
//             "outputs" : [],
//             "type" : "function"
//          },
//          {
//             "id" : 528,
//             "inputs" : [],
//             "name" : "clear_errors",
//             "outputs" : [],
//             "type" : "function"
//          }
//       ],
//       "name" : "axis1",
//       "type" : "object"
//    },
//    {
//       "members" : [
//          {
//             "access" : "rw",
//             "id" : 529,
//             "name" : "error",
//             "type" : "int32"
//          },
//          {
//             "members" : [
//                {
//                   "access" : "r",
//                   "id" : 530,
//                   "name" : "baud_rate",
//                   "type" : "uint32"
//                },
//                {
//                   "access" : "rw",
//                   "id" : 531,
//                   "name" : "protocol",
//                   "type" : "int32"
//                }
//             ],
//             "name" : "config",
//             "type" : "object"
//          },
//          {
//             "id" : 532,
//             "inputs" : [
//                {
//                   "access" : "rw",
//                   "id" : 533,
//                   "name" : "baudRate",
//                   "type" : "uint32"
//                }
//             ],
//             "name" : "set_baud_rate",
//             "outputs" : [],
//             "type" : "function"
//          }
//       ],
//       "name" : "can",
//       "type" : "object"
//    },
//    {
//       "access" : "rw",
//       "id" : 534,
//       "name" : "test_property",
//       "type" : "uint32"
//    },
//    {
//       "id" : 535,
//       "inputs" : [
//          {
//             "access" : "rw",
//             "id" : 536,
//             "name" : "delta",
//             "type" : "int32"
//          }
//       ],
//       "name" : "test_function",
//       "outputs" : [
//          {
//             "access" : "r",
//             "id" : 537,
//             "name" : "cnt",
//             "type" : "int32"
//          }
//       ],
//       "type" : "function"
//    },
//    {
//       "id" : 538,
//       "inputs" : [
//          {
//             "access" : "rw",
//             "id" : 539,
//             "name" : "index",
//             "type" : "uint32"
//          }
//       ],
//       "name" : "get_oscilloscope_val",
//       "outputs" : [
//          {
//             "access" : "r",
//             "id" : 540,
//             "name" : "val",
//             "type" : "float"
//          }
//       ],
//       "type" : "function"
//    },
//    {
//       "id" : 541,
//       "inputs" : [
//          {
//             "access" : "rw",
//             "id" : 542,
//             "name" : "gpio",
//             "type" : "uint32"
//          }
//       ],
//       "name" : "get_adc_voltage",
//       "outputs" : [
//          {
//             "access" : "r",
//             "id" : 543,
//             "name" : "voltage",
//             "type" : "float"
//          }
//       ],
//       "type" : "function"
//    },
//    {
//       "id" : 544,
//       "inputs" : [],
//       "name" : "save_configuration",
//       "outputs" : [],
//       "type" : "function"
//    },
//    {
//       "id" : 545,
//       "inputs" : [],
//       "name" : "erase_configuration",
//       "outputs" : [],
//       "type" : "function"
//    },
//    {
//       "id" : 546,
//       "inputs" : [],
//       "name" : "reboot",
//       "outputs" : [],
//       "type" : "function"
//    },
//    {
//       "id" : 547,
//       "inputs" : [],
//       "name" : "enter_dfu_mode",
//       "outputs" : [],
//       "type" : "function"
//    }
// ]


// ______________________ OLD !!!!! ______________________________ //

/*
odrive allows to send commands such as
  vbus_voltage
  axis0.error
  axis0.encoder.vel_estimate
  axis0.motor.current_meas_phC
  axis0.controller.vel_setpoint
Search for the right endpoint id in the configuration file

[
  {"name":"","id":0,"type":"json","access":"r"},
  {"name":"vbus_voltage","id":1,"type":"float","access":"r"},
  {"name":"serial_number","id":2,"type":"uint64","access":"r"},
  {"name":"hw_version_major","id":3,"type":"uint8","access":"r"},
  {"name":"hw_version_minor","id":4,"type":"uint8","access":"r"},
  {"name":"hw_version_variant","id":5,"type":"uint8","access":"r"},
  {"name":"fw_version_major","id":6,"type":"uint8","access":"r"},
  {"name":"fw_version_minor","id":7,"type":"uint8","access":"r"},
  {"name":"fw_version_revision","id":8,"type":"uint8","access":"r"},
  {"name":"fw_version_unreleased","id":9,"type":"uint8","access":"r"},
  {"name":"user_config_loaded","id":10,"type":"bool","access":"r"},
  {"name":"brake_resistor_armed","id":11,"type":"bool","access":"r"},
  {"name":"system_stats","type":"object","members":[
    {"name":"uptime","id":12,"type":"uint32","access":"r"},
    {"name":"min_heap_space","id":13,"type":"uint32","access":"r"},
    {"name":"min_stack_space_axis0","id":14,"type":"uint32","access":"r"},
    {"name":"min_stack_space_axis1","id":15,"type":"uint32","access":"r"},
    {"name":"min_stack_space_comms","id":16,"type":"uint32","access":"r"},
    {"name":"min_stack_space_usb","id":17,"type":"uint32","access":"r"},
    {"name":"min_stack_space_uart","id":18,"type":"uint32","access":"r"},
    {"name":"min_stack_space_usb_irq","id":19,"type":"uint32","access":"r"},
    {"name":"min_stack_space_startup","id":20,"type":"uint32","access":"r"},
    {"name":"usb","type":"object","members":[ 
      {"name":"rx_cnt","id":21,"type":"uint32","access":"r"},
      {"name":"tx_cnt","id":22,"type":"uint32","access":"r"},
      {"name":"tx_overrun_cnt","id":23,"type":"uint32","access":"r"}
    ]},
    {"name":"i2c","type":"object","members":[
      {"name":"addr","id":24,"type":"uint8","access":"r"},
      {"name":"addr_match_cnt","id":25,"type":"uint32","access":"r"},
      {"name":"rx_cnt","id":26,"type":"uint32","access":"r"},
      {"name":"error_cnt","id":27,"type":"uint32","access":"r"}
    ]}
  ]},
  {"name":"config","type":"object","members":[
    {"name":"brake_resistance","id":28,"type":"float","access":"rw"},
    {"name":"enable_uart","id":29,"type":"bool","access":"rw"},
    {"name":"enable_i2c_instead_of_can","id":30,"type":"bool","access":"rw"},
    {"name":"enable_ascii_protocol_on_usb","id":31,"type":"bool","access":"rw"},
    {"name":"dc_bus_undervoltage_trip_level","id":32,"type":"float","access":"rw"},
    {"name":"dc_bus_overvoltage_trip_level","id":33,"type":"float","access":"rw"},
    {"name":"gpio1_pwm_mapping","type":"object","members":[
      {"name":"endpoint","id":34,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":35,"type":"float","access":"rw"},
      {"name":"max","id":36,"type":"float","access":"rw"}
    ]},
    {"name":"gpio2_pwm_mapping","type":"object","members":[
      {"name":"endpoint","id":37,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":38,"type":"float","access":"rw"},
      {"name":"max","id":39,"type":"float","access":"rw"}
    ]},
    {"name":"gpio3_pwm_mapping","type":"object","members":[
      {"name":"endpoint","id":40,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":41,"type":"float","access":"rw"},
      {"name":"max","id":42,"type":"float","access":"rw"}
    ]},
    {"name":"gpio4_pwm_mapping","type":"object","members":[
      {"name":"endpoint","id":43,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":44,"type":"float","access":"rw"},
      {"name":"max","id":45,"type":"float","access":"rw"}
    ]},
    {"name":"gpio3_analog_mapping","type":"object","members":[
      {"name":"endpoint","id":46,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":47,"type":"float","access":"rw"},
      {"name":"max","id":48,"type":"float","access":"rw"}
    ]},
    {"name":"gpio4_analog_mapping","type":"object","members":[
      {"name":"endpoint","id":49,"type":"endpoint_ref","access":"rw"},
      {"name":"min","id":50,"type":"float","access":"rw"},
      {"name":"max","id":51,"type":"float","access":"rw"}
    ]}
  ]},
  {"name":"axis0","type":"object","members":[
    {"name":"error","id":52,"type":"uint16","access":"rw"},
    {"name":"step_dir_active","id":53,"type":"bool","access":"r"},
    {"name":"current_state","id":54,"type":"uint8","access":"r"},
    {"name":"requested_state","id":55,"type":"uint8","access":"rw"},
    {"name":"loop_counter","id":56,"type":"uint32","access":"r"},
    {"name":"lockin_state","id":57,"type":"uint8","access":"r"},
    {"name":"config","type":"object","members":[
      {"name":"startup_motor_calibration","id":58,"type":"bool","access":"rw"},
      {"name":"startup_encoder_index_search","id":59,"type":"bool","access":"rw"},
      {"name":"startup_encoder_offset_calibration","id":60,"type":"bool","access":"rw"},
      {"name":"startup_closed_loop_control","id":61,"type":"bool","access":"rw"},
      {"name":"startup_sensorless_control","id":62,"type":"bool","access":"rw"},
      {"name":"enable_step_dir","id":63,"type":"bool","access":"rw"},
      {"name":"counts_per_step","id":64,"type":"float","access":"rw"},
      {"name":"watchdog_timeout","id":65,"type":"float","access":"rw"},
      {"name":"step_gpio_pin","id":66,"type":"uint16","access":"rw"},
      {"name":"dir_gpio_pin","id":67,"type":"uint16","access":"rw"},
      {"name":"calibration_lockin","type":"object","members":[
        {"name":"current","id":68,"type":"float","access":"rw"},
        {"name":"ramp_time","id":69,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":70,"type":"float","access":"rw"},
        {"name":"accel","id":71,"type":"float","access":"rw"},
        {"name":"vel","id":72,"type":"float","access":"rw"}
      ]},
      {"name":"sensorless_ramp","type":"object","members":[
        {"name":"current","id":73,"type":"float","access":"rw"},
        {"name":"ramp_time","id":74,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":75,"type":"float","access":"rw"},
        {"name":"accel","id":76,"type":"float","access":"rw"},
        {"name":"vel","id":77,"type":"float","access":"rw"},
        {"name":"finish_distance","id":78,"type":"float","access":"rw"},
        {"name":"finish_on_vel","id":79,"type":"bool","access":"rw"},
        {"name":"finish_on_distance","id":80,"type":"bool","access":"rw"},
        {"name":"finish_on_enc_idx","id":81,"type":"bool","access":"rw"}
      ]},
      {"name":"general_lockin","type":"object","members":[
        {"name":"current","id":82,"type":"float","access":"rw"},
        {"name":"ramp_time","id":83,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":84,"type":"float","access":"rw"},
        {"name":"accel","id":85,"type":"float","access":"rw"},
        {"name":"vel","id":86,"type":"float","access":"rw"},
        {"name":"finish_distance","id":87,"type":"float","access":"rw"},
        {"name":"finish_on_vel","id":88,"type":"bool","access":"rw"},
        {"name":"finish_on_distance","id":89,"type":"bool","access":"rw"},
        {"name":"finish_on_enc_idx","id":90,"type":"bool","access":"rw"}
      ]}
    ]},
    {"name":"motor","type":"object","members":[
      {"name":"error","id":91,"type":"uint16","access":"rw"},
      {"name":"armed_state","id":92,"type":"uint8","access":"r"},
      {"name":"is_calibrated","id":93,"type":"bool","access":"r"},
      {"name":"current_meas_phB","id":94,"type":"float","access":"r"},
      {"name":"current_meas_phC","id":95,"type":"float","access":"r"},
      {"name":"DC_calib_phB","id":96,"type":"float","access":"rw"},
      {"name":"DC_calib_phC","id":97,"type":"float","access":"rw"},
      {"name":"phase_current_rev_gain","id":98,"type":"float","access":"rw"},
      {"name":"thermal_current_lim","id":99,"type":"float","access":"r"},
      {"name":"get_inverter_temp","id":100,"type":"function","inputs":[],"outputs":[{"name":"result","id":101,"type":"float","access":"rw"}]},
      {"name":"current_control","type":"object","members":[
        {"name":"p_gain","id":102,"type":"float","access":"rw"},
        {"name":"i_gain","id":103,"type":"float","access":"rw"},
        {"name":"v_current_control_integral_d","id":104,"type":"float","access":"rw"},
        {"name":"v_current_control_integral_q","id":105,"type":"float","access":"rw"},
        {"name":"Ibus","id":106,"type":"float","access":"rw"},
        {"name":"final_v_alpha","id":107,"type":"float","access":"rw"},
        {"name":"final_v_beta","id":108,"type":"float","access":"rw"},
        {"name":"Iq_setpoint","id":109,"type":"float","access":"rw"},
        {"name":"Iq_measured","id":110,"type":"float","access":"rw"},
        {"name":"Id_measured","id":111,"type":"float","access":"rw"},
        {"name":"I_measured_report_filter_k","id":112,"type":"float","access":"rw"},
        {"name":"max_allowed_current","id":113,"type":"float","access":"r"},
        {"name":"overcurrent_trip_level","id":114,"type":"float","access":"r"}
      ]},
      {"name":"gate_driver","type":"object","members":[
        {"name":"drv_fault","id":115,"type":"uint16","access":"r"}
      ]},
      {"name":"timing_log","type":"object","members":[
        {"name":"TIMING_LOG_GENERAL","id":116,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ADC_CB_I","id":117,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ADC_CB_DC","id":118,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_MEAS_R","id":119,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_MEAS_L","id":120,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ENC_CALIB","id":121,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_IDX_SEARCH","id":122,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_FOC_VOLTAGE","id":123,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_FOC_CURRENT","id":124,"type":"uint16","access":"r"}
      ]},
      {"name":"config","type":"object","members":[
        {"name":"pre_calibrated","id":125,"type":"bool","access":"rw"},
        {"name":"pole_pairs","id":126,"type":"int32","access":"rw"},
        {"name":"calibration_current","id":127,"type":"float","access":"rw"},
        {"name":"resistance_calib_max_voltage","id":128,"type":"float","access":"rw"},
        {"name":"phase_inductance","id":129,"type":"float","access":"rw"},
        {"name":"phase_resistance","id":130,"type":"float","access":"rw"},
        {"name":"direction","id":131,"type":"int32","access":"rw"},
        {"name":"motor_type","id":132,"type":"uint8","access":"rw"},
        {"name":"current_lim","id":133,"type":"float","access":"rw"},
        {"name":"current_lim_tolerance","id":134,"type":"float","access":"rw"},
        {"name":"inverter_temp_limit_lower","id":135,"type":"float","access":"rw"},
        {"name":"inverter_temp_limit_upper","id":136,"type":"float","access":"rw"},
        {"name":"requested_current_range","id":137,"type":"float","access":"rw"},
        {"name":"current_control_bandwidth","id":138,"type":"float","access":"rw"}
      ]}
    ]},
    {"name":"controller","type":"object","members":[
      {"name":"error","id":139,"type":"uint8","access":"rw"},
      {"name":"pos_setpoint","id":140,"type":"float","access":"rw"},
      {"name":"vel_setpoint","id":141,"type":"float","access":"rw"},
      {"name":"vel_integrator_current","id":142,"type":"float","access":"rw"},
      {"name":"current_setpoint","id":143,"type":"float","access":"rw"},
      {"name":"vel_ramp_target","id":144,"type":"float","access":"rw"},
      {"name":"vel_ramp_enable","id":145,"type":"bool","access":"rw"},
      {"name":"config","type":"object","members":[
        {"name":"control_mode","id":146,"type":"uint8","access":"rw"},
        {"name":"pos_gain","id":147,"type":"float","access":"rw"},
        {"name":"vel_gain","id":148,"type":"float","access":"rw"},
        {"name":"vel_integrator_gain","id":149,"type":"float","access":"rw"},
        {"name":"vel_limit","id":150,"type":"float","access":"rw"},
        {"name":"vel_limit_tolerance","id":151,"type":"float","access":"rw"},
        {"name":"vel_ramp_rate","id":152,"type":"float","access":"rw"},
        {"name":"setpoints_in_cpr","id":153,"type":"bool","access":"rw"}
      ]},
      {"name":"set_pos_setpoint","id":154,"type":"function","inputs":[{"name":"pos_setpoint","id":155,"type":"float","access":"rw"},{"name":"vel_feed_forward","id":156,"type":"float","access":"rw"},{"name":"current_feed_forward","id":157,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"set_vel_setpoint","id":158,"type":"function","inputs":[{"name":"vel_setpoint","id":159,"type":"float","access":"rw"},{"name":"current_feed_forward","id":160,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"set_current_setpoint","id":161,"type":"function","inputs":[{"name":"current_setpoint","id":162,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"move_to_pos","id":163,"type":"function","inputs":[{"name":"pos_setpoint","id":164,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"move_incremental","id":165,"type":"function","inputs":[{"name":"displacement","id":166,"type":"float","access":"rw"},{"name":"from_goal_point","id":167,"type":"bool","access":"rw"}],"outputs":[]},
      {"name":"start_anticogging_calibration","id":168,"type":"function","inputs":[],"outputs":[]}
    ]},
    {"name":"encoder","type":"object","members":[
      {"name":"error","id":169,"type":"uint8","access":"rw"},
      {"name":"is_ready","id":170,"type":"bool","access":"rw"},
      {"name":"index_found","id":171,"type":"bool","access":"rw"},
      {"name":"shadow_count","id":172,"type":"int32","access":"rw"},
      {"name":"count_in_cpr","id":173,"type":"int32","access":"rw"},
      {"name":"interpolation","id":174,"type":"float","access":"rw"},
      {"name":"phase","id":175,"type":"float","access":"r"},
      {"name":"pos_estimate","id":176,"type":"float","access":"rw"},
      {"name":"pos_cpr","id":177,"type":"float","access":"rw"},
      {"name":"hall_state","id":178,"type":"uint8","access":"r"},
      {"name":"vel_estimate","id":179,"type":"float","access":"rw"},
      {"name":"calib_scan_response","id":180,"type":"float","access":"r"},
      {"name":"config","type":"object","members":[
        {"name":"mode","id":181,"type":"uint8","access":"rw"},
        {"name":"use_index","id":182,"type":"bool","access":"rw"},
        {"name":"find_idx_on_lockin_only","id":183,"type":"bool","access":"rw"},
        {"name":"pre_calibrated","id":184,"type":"bool","access":"rw"},
        {"name":"zero_count_on_find_idx","id":185,"type":"bool","access":"rw"},
        {"name":"cpr","id":186,"type":"int32","access":"rw"},
        {"name":"offset","id":187,"type":"int32","access":"rw"},
        {"name":"offset_float","id":188,"type":"float","access":"rw"},
        {"name":"enable_phase_interpolation","id":189,"type":"bool","access":"rw"},
        {"name":"bandwidth","id":190,"type":"float","access":"rw"},
        {"name":"calib_range","id":191,"type":"float","access":"rw"},
        {"name":"calib_scan_distance","id":192,"type":"float","access":"rw"},
        {"name":"calib_scan_omega","id":193,"type":"float","access":"rw"},
        {"name":"idx_search_unidirectional","id":194,"type":"bool","access":"rw"},
        {"name":"ignore_illegal_hall_state","id":195,"type":"bool","access":"rw"}
      ]},
      {"name":"set_linear_count","id":196,"type":"function","inputs":[{"name":"count","id":197,"type":"int32","access":"rw"}],"outputs":[]}
    ]},
    {"name":"sensorless_estimator","type":"object","members":[
      {"name":"error","id":198,"type":"uint8","access":"rw"},
      {"name":"phase","id":199,"type":"float","access":"rw"},
      {"name":"pll_pos","id":200,"type":"float","access":"rw"},
      {"name":"vel_estimate","id":201,"type":"float","access":"rw"},
      {"name":"config","type":"object","members":[
        {"name":"observer_gain","id":202,"type":"float","access":"rw"},
        {"name":"pll_bandwidth","id":203,"type":"float","access":"rw"},
        {"name":"pm_flux_linkage","id":204,"type":"float","access":"rw"}
      ]}
    ]},
    {"name":"trap_traj","type":"object","members":[
      {"name":"config","type":"object","members":[
        {"name":"vel_limit","id":205,"type":"float","access":"rw"},
        {"name":"accel_limit","id":206,"type":"float","access":"rw"},
        {"name":"decel_limit","id":207,"type":"float","access":"rw"},
        {"name":"A_per_css","id":208,"type":"float","access":"rw"}
      ]}
    ]},
    {"name":"watchdog_feed","id":209,"type":"function","inputs":[],"outputs":[]}
  ]},
  
  {"name":"axis1","type":"object","members":[
    {"name":"error","id":210,"type":"uint16","access":"rw"},
    {"name":"step_dir_active","id":211,"type":"bool","access":"r"},
    {"name":"current_state","id":212,"type":"uint8","access":"r"},
    {"name":"requested_state","id":213,"type":"uint8","access":"rw"},
    {"name":"loop_counter","id":214,"type":"uint32","access":"r"},
    {"name":"lockin_state","id":215,"type":"uint8","access":"r"},
    {"name":"config","type":"object","members":[
      {"name":"startup_motor_calibration","id":216,"type":"bool","access":"rw"},
      {"name":"startup_encoder_index_search","id":217,"type":"bool","access":"rw"},
      {"name":"startup_encoder_offset_calibration","id":218,"type":"bool","access":"rw"},
      {"name":"startup_closed_loop_control","id":219,"type":"bool","access":"rw"},
      {"name":"startup_sensorless_control","id":220,"type":"bool","access":"rw"},
      {"name":"enable_step_dir","id":221,"type":"bool","access":"rw"},
      {"name":"counts_per_step","id":222,"type":"float","access":"rw"},
      {"name":"watchdog_timeout","id":223,"type":"float","access":"rw"},
      {"name":"step_gpio_pin","id":224,"type":"uint16","access":"rw"},
      {"name":"dir_gpio_pin","id":225,"type":"uint16","access":"rw"},
      {"name":"calibration_lockin","type":"object","members":[
        {"name":"current","id":226,"type":"float","access":"rw"},
        {"name":"ramp_time","id":227,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":228,"type":"float","access":"rw"},
        {"name":"accel","id":229,"type":"float","access":"rw"},
        {"name":"vel","id":230,"type":"float","access":"rw"}
      ]},
      {"name":"sensorless_ramp","type":"object","members":[
        {"name":"current","id":231,"type":"float","access":"rw"},
        {"name":"ramp_time","id":232,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":233,"type":"float","access":"rw"},
        {"name":"accel","id":234,"type":"float","access":"rw"},
        {"name":"vel","id":235,"type":"float","access":"rw"},
        {"name":"finish_distance","id":236,"type":"float","access":"rw"},
        {"name":"finish_on_vel","id":237,"type":"bool","access":"rw"},
        {"name":"finish_on_distance","id":238,"type":"bool","access":"rw"},
        {"name":"finish_on_enc_idx","id":239,"type":"bool","access":"rw"}
      ]},
      {"name":"general_lockin","type":"object","members":[
        {"name":"current","id":240,"type":"float","access":"rw"},
        {"name":"ramp_time","id":241,"type":"float","access":"rw"},
        {"name":"ramp_distance","id":242,"type":"float","access":"rw"},
        {"name":"accel","id":243,"type":"float","access":"rw"},
        {"name":"vel","id":244,"type":"float","access":"rw"},
        {"name":"finish_distance","id":245,"type":"float","access":"rw"},
        {"name":"finish_on_vel","id":246,"type":"bool","access":"rw"},
        {"name":"finish_on_distance","id":247,"type":"bool","access":"rw"},
        {"name":"finish_on_enc_idx","id":248,"type":"bool","access":"rw"}
      ]}
    ]},
    {"name":"motor","type":"object","members":[
      {"name":"error","id":249,"type":"uint16","access":"rw"},
      {"name":"armed_state","id":250,"type":"uint8","access":"r"},
      {"name":"is_calibrated","id":251,"type":"bool","access":"r"},
      {"name":"current_meas_phB","id":252,"type":"float","access":"r"},
      {"name":"current_meas_phC","id":253,"type":"float","access":"r"},
      {"name":"DC_calib_phB","id":254,"type":"float","access":"rw"},
      {"name":"DC_calib_phC","id":255,"type":"float","access":"rw"},
      {"name":"phase_current_rev_gain","id":256,"type":"float","access":"rw"},
      {"name":"thermal_current_lim","id":257,"type":"float","access":"r"},
      {"name":"get_inverter_temp","id":258,"type":"function","inputs":[],"outputs":[{"name":"result","id":259,"type":"float","access":"rw"}]},
      {"name":"current_control","type":"object","members":[
        {"name":"p_gain","id":260,"type":"float","access":"rw"},
        {"name":"i_gain","id":261,"type":"float","access":"rw"},
        {"name":"v_current_control_integral_d","id":262,"type":"float","access":"rw"},
        {"name":"v_current_control_integral_q","id":263,"type":"float","access":"rw"},
        {"name":"Ibus","id":264,"type":"float","access":"rw"},
        {"name":"final_v_alpha","id":265,"type":"float","access":"rw"},
        {"name":"final_v_beta","id":266,"type":"float","access":"rw"},
        {"name":"Iq_setpoint","id":267,"type":"float","access":"rw"},
        {"name":"Iq_measured","id":268,"type":"float","access":"rw"},
        {"name":"Id_measured","id":269,"type":"float","access":"rw"},
        {"name":"I_measured_report_filter_k","id":270,"type":"float","access":"rw"},
        {"name":"max_allowed_current","id":271,"type":"float","access":"r"},
        {"name":"overcurrent_trip_level","id":272,"type":"float","access":"r"}
      ]},
      {"name":"gate_driver","type":"object","members":[
        {"name":"drv_fault","id":273,"type":"uint16","access":"r"}
      ]},
      {"name":"timing_log","type":"object","members":[
        {"name":"TIMING_LOG_GENERAL","id":274,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ADC_CB_I","id":275,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ADC_CB_DC","id":276,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_MEAS_R","id":277,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_MEAS_L","id":278,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_ENC_CALIB","id":279,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_IDX_SEARCH","id":280,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_FOC_VOLTAGE","id":281,"type":"uint16","access":"r"},
        {"name":"TIMING_LOG_FOC_CURRENT","id":282,"type":"uint16","access":"r"}
      ]},
      {"name":"config","type":"object","members":[  
        {"name":"pre_calibrated","id":283,"type":"bool","access":"rw"},
        {"name":"pole_pairs","id":284,"type":"int32","access":"rw"},
        {"name":"calibration_current","id":285,"type":"float","access":"rw"},
        {"name":"resistance_calib_max_voltage","id":286,"type":"float","access":"rw"},
        {"name":"phase_inductance","id":287,"type":"float","access":"rw"},
        {"name":"phase_resistance","id":288,"type":"float","access":"rw"},
        {"name":"direction","id":289,"type":"int32","access":"rw"},
        {"name":"motor_type","id":290,"type":"uint8","access":"rw"},
        {"name":"current_lim","id":291,"type":"float","access":"rw"},
        {"name":"current_lim_tolerance","id":292,"type":"float","access":"rw"},
        {"name":"inverter_temp_limit_lower","id":293,"type":"float","access":"rw"},
        {"name":"inverter_temp_limit_upper","id":294,"type":"float","access":"rw"},
        {"name":"requested_current_range","id":295,"type":"float","access":"rw"},
        {"name":"current_control_bandwidth","id":296,"type":"float","access":"rw"}
      ]}
    ]},
    {"name":"controller","type":"object","members":[
      {"name":"error","id":297,"type":"uint8","access":"rw"},
      {"name":"pos_setpoint","id":298,"type":"float","access":"rw"},
      {"name":"vel_setpoint","id":299,"type":"float","access":"rw"},
      {"name":"vel_integrator_current","id":300,"type":"float","access":"rw"},
      {"name":"current_setpoint","id":301,"type":"float","access":"rw"},
      {"name":"vel_ramp_target","id":302,"type":"float","access":"rw"},
      {"name":"vel_ramp_enable","id":303,"type":"bool","access":"rw"},
      {"name":"config","type":"object","members":[
        {"name":"control_mode","id":304,"type":"uint8","access":"rw"},
        {"name":"pos_gain","id":305,"type":"float","access":"rw"},
        {"name":"vel_gain","id":306,"type":"float","access":"rw"},
        {"name":"vel_integrator_gain","id":307,"type":"float","access":"rw"},
        {"name":"vel_limit","id":308,"type":"float","access":"rw"},
        {"name":"vel_limit_tolerance","id":309,"type":"float","access":"rw"},
        {"name":"vel_ramp_rate","id":310,"type":"float","access":"rw"},
        {"name":"setpoints_in_cpr","id":311,"type":"bool","access":"rw"}
      ]},
      {"name":"set_pos_setpoint","id":312,"type":"function","inputs":[{"name":"pos_setpoint","id":313,"type":"float","access":"rw"},{"name":"vel_feed_forward","id":314,"type":"float","access":"rw"},{"name":"current_feed_forward","id":315,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"set_vel_setpoint","id":316,"type":"function","inputs":[{"name":"vel_setpoint","id":317,"type":"float","access":"rw"},{"name":"current_feed_forward","id":318,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"set_current_setpoint","id":319,"type":"function","inputs":[{"name":"current_setpoint","id":320,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"move_to_pos","id":321,"type":"function","inputs":[{"name":"pos_setpoint","id":322,"type":"float","access":"rw"}],"outputs":[]},
      {"name":"move_incremental","id":323,"type":"function","inputs":[{"name":"displacement","id":324,"type":"float","access":"rw"},{"name":"from_goal_point","id":325,"type":"bool","access":"rw"}],"outputs":[]},
      {"name":"start_anticogging_calibration","id":326,"type":"function","inputs":[],"outputs":[]}
    ]},
    {"name":"encoder","type":"object","members":[
      {"name":"error","id":327,"type":"uint8","access":"rw"},
      {"name":"is_ready","id":328,"type":"bool","access":"rw"},
      {"name":"index_found","id":329,"type":"bool","access":"rw"},
      {"name":"shadow_count","id":330,"type":"int32","access":"rw"},
      {"name":"count_in_cpr","id":331,"type":"int32","access":"rw"},
      {"name":"interpolation","id":332,"type":"float","access":"rw"},
      {"name":"phase","id":333,"type":"float","access":"r"},
      {"name":"pos_estimate","id":334,"type":"float","access":"rw"},
      {"name":"pos_cpr","id":335,"type":"float","access":"rw"},
      {"name":"hall_state","id":336,"type":"uint8","access":"r"},
      {"name":"vel_estimate","id":337,"type":"float","access":"rw"},
      {"name":"calib_scan_response","id":338,"type":"float","access":"r"},
      {"name":"config","type":"object","members":[
        {"name":"mode","id":339,"type":"uint8","access":"rw"},
        {"name":"use_index","id":340,"type":"bool","access":"rw"},
        {"name":"find_idx_on_lockin_only","id":341,"type":"bool","access":"rw"},
        {"name":"pre_calibrated","id":342,"type":"bool","access":"rw"},
        {"name":"zero_count_on_find_idx","id":343,"type":"bool","access":"rw"},
        {"name":"cpr","id":344,"type":"int32","access":"rw"},
        {"name":"offset","id":345,"type":"int32","access":"rw"},
        {"name":"offset_float","id":346,"type":"float","access":"rw"},
        {"name":"enable_phase_interpolation","id":347,"type":"bool","access":"rw"},
        {"name":"bandwidth","id":348,"type":"float","access":"rw"},
        {"name":"calib_range","id":349,"type":"float","access":"rw"},
        {"name":"calib_scan_distance","id":350,"type":"float","access":"rw"},
        {"name":"calib_scan_omega","id":351,"type":"float","access":"rw"},
        {"name":"idx_search_unidirectional","id":352,"type":"bool","access":"rw"},
        {"name":"ignore_illegal_hall_state","id":353,"type":"bool","access":"rw"}
      ]},
      {"name":"set_linear_count","id":354,"type":"function","inputs":[{"name":"count","id":355,"type":"int32","access":"rw"}],"outputs":[]}
    ]},
    {"name":"sensorless_estimator","type":"object","members":[
      {"name":"error","id":356,"type":"uint8","access":"rw"},
      {"name":"phase","id":357,"type":"float","access":"rw"},
      {"name":"pll_pos","id":358,"type":"float","access":"rw"},
      {"name":"vel_estimate","id":359,"type":"float","access":"rw"},
      {"name":"config","type":"object","members":[
        {"name":"observer_gain","id":360,"type":"float","access":"rw"},
        {"name":"pll_bandwidth","id":361,"type":"float","access":"rw"},
        {"name":"pm_flux_linkage","id":362,"type":"float","access":"rw"}
      ]}
    ]},    
    {"name":"trap_traj","type":"object","members":[
      {"name":"config","type":"object","members":[
        {"name":"vel_limit","id":363,"type":"float","access":"rw"},
        {"name":"accel_limit","id":364,"type":"float","access":"rw"},
        {"name":"decel_limit","id":365,"type":"float","access":"rw"},
        {"name":"A_per_css","id":366,"type":"float","access":"rw"}
      ]}
    ]},
    {"name":"watchdog_feed","id":367,"type":"function","inputs":[],"outputs":[]}
  ]},
  {"name":"can","type":"object","members":[
    {"name":"node_id","id":368,"type":"uint8","access":"r"},
    {"name":"TxMailboxCompleteCallbackCnt","id":369,"type":"uint32","access":"r"},
    {"name":"TxMailboxAbortCallbackCnt","id":370,"type":"uint32","access":"r"},
    {"name":"received_msg_cnt","id":371,"type":"uint32","access":"r"},
    {"name":"received_ack","id":372,"type":"uint32","access":"r"},
    {"name":"unexpected_errors","id":373,"type":"uint32","access":"r"},
    {"name":"unhandled_messages","id":374,"type":"uint32","access":"r"},
  ]},
  {"name":"test_property","id":375,"type":"uint32","access":"rw"}
  {"name":"test_function","id":376,"type":"function","inputs":[{"name":"delta","id":377,"type":"int32","access":"rw"}],"outputs":[{"name":"result","id":378,"type":"int32","access":"rw"}]},
  {"name":"get_oscilloscope_val","id":379,"type":"function","inputs":[{"name":"index","id":380,"type":"uint32","access":"rw"}],"outputs":[{"name":"result","id":381,"type":"float","access":"rw"}]},
  {"name":"get_adc_voltage","id":382,"type":"function","inputs":[{"name":"gpio","id":383,"type":"uint32","access":"rw"}],"outputs":[{"name":"result","id":384,"type":"float","access":"rw"}]},
  {"name":"save_configuration","id":385,"type":"function","inputs":[],"outputs":[]},
  {"name":"erase_configuration","id":386,"type":"function","inputs":[],"outputs":[]},
  {"name":"reboot","id":387,"type":"function","inputs":[],"outputs":[]},
  {"name":"enter_dfu_mode","id":388,"type":"function","inputs":[],"outputs":[]},

]

*/
