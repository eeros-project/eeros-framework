#include "odrive/odrive.hpp"
#include <jsoncpp/json/json.h>

using namespace odrive;

ODrive::ODrive(const float ticks) : encoderTicksPerRad(ticks) {
    libusbContext = NULL;
    ep = new ODriveEP();
}

ODrive::~ODrive() {
  delete ep;
  if (libusbContext) { 
    libusb_exit(libusbContext); 
    libusbContext = NULL;
  }
}

int ODrive::init(uint64_t serialNumber) {
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

  int ret = 1;
  for (size_t i = 0; i < device_count; ++i) {
    libusb_device *device = usb_device_list[i];
    libusb_device_descriptor desc = {0};

    res = libusb_get_device_descriptor(device, &desc);
    if (res != LIBUSB_SUCCESS) {
      std::cerr << "error getting device descriptor" << std::endl;
      continue;
    }
    /* Check USB devicei ID */
    if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {	  
      libusb_device_handle *device_handle;
      if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
        std::cerr << "error opening USB device" << std::endl;
        continue;
      }

      libusb_reset_device(device_handle);
      std::cout << "reset device" << std::endl;
      sleep(1);

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
          strea << std::uppercase << std::hex << serialNumber;
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

/**
 *  Read JSON file from target
 */
int ODrive::getJson() {
    commBuffer rx, tx;
    int len, address = 0;
    std::string str;
    std::cout << "reading target configuration" << std::endl;
    do {
        ep->endpointRequest(0, rx, len, tx, true, 512, true, address);
        address = address + len;
        str.append((const char *)&rx[0], (size_t)len);
    } while (len > 0);
    Json::Reader reader;
    bool res = reader.parse(str, json);
    if (!res) {
        std::cerr << "Error parsing json!" << std::endl;
        return 1;
    }
    return 0;
}

