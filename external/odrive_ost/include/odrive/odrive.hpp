#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <iostream>
#include <sstream>
#include <getopt.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <endian.h>
#include <jsoncpp/json/json.h>
#include "odriveEP.hpp"


namespace odrive {

typedef std::vector<uint8_t> commBuffer;

class ODrive {
 public:
  ODrive(const float ticks);
  ~ODrive();

  int init(uint64_t serialNumber);
  int getJson();
  ODriveEP* ep;
  Json::Value json;

 private:
  float encoderTicksPerRad;
  libusb_context* libusbContext;
};

}

#endif /* ODRIVE_HPP_ */
