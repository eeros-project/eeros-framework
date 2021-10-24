#ifndef ORG_EEROS_HAL_PIXYCAM_HPP_
#define ORG_EEROS_HAL_PIXYCAM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <pixy2/libpixyusb2.h>
#include <unistd.h>
#include <atomic>

#define nrDots 4

using namespace eeros::logger;
using namespace eeros::math;

namespace eeros {
namespace hal {

/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::PixyCamInput class. 
 * Do not use it directly. Fetch and build the pixy2 library.
 * Unfortunately, it misses suitable make and install targets.
 * You have to manually copy the proper header files to your 
 * install directory. As the library is not built for relocatable 
 * code, eeros cannot be built with pixy code in it and all the 
 * code has been placed into this header file.
 *
 * @since v1.3
 */

class PixyCam : public eeros::Thread {
 public:
  explicit PixyCam(std::string dev, int priority) : starting(true), running(false), log(Logger::getLogger()) {
    log.info() << "Connecting to Pixy2";
    auto res = pixy.init();
    if (res < 0) {
      log.warn() << "no connection to Pixy2 possible";
      return;
    } else log.info() << "Pixy2 connected";
    pixy.getVersion();
    log.info() << "Pixy2 version = " << pixy.version;
    // Set Pixy2 to color connected components program 
    pixy.changeProg("color_connected_components");
    starting = false;
  }
  
  /**
   * Destructs the thread.
   */
  virtual ~PixyCam() { 
    pixy.setLamp(0, 0);
    running = false; 
    join(); 
  }

  /**
   * Returns the position of the camera, with respect to the middle of the 
   * markers pattern, with lowpass filter
   */
  virtual Vector3 getPos() {
    return output;
  }
  
  /**
   * Returns the position of the camera, with respect to the middle of the markers pattern 
   */
  virtual Vector3 getPosRaw() {
    return outputRaw;
  }
  
  /**
   * Returns the position of the detected dots on the markers pattern in pixels 
   */
  virtual Matrix<nrDots,2,double> getDots() {
    return pos;
  }

  /**
   * Returns the height of the camera to the markers pattern
   */
  virtual double getHeight() {
    return height;
  }
  
  /**
   * Returns true if data are valid, i.e. if all markers are within the field of view of the camera
   */
  virtual double isDataValid() {
    return valid;
  }
  
  /**
   * Returns the nu,ber of markers detected by the camera
   */
  virtual int getNofBlocks() {
    return nofBlocks;
  }

  /**
   * Sets the lamp of the camera
   * @param white: white light on if true
   * @param rgb: rgb light on if true
   */
  virtual void setLamp(bool white, bool rgb) {
    pixy.setLamp(white, rgb);
  }

 private:
    
  /**
   * Gets sensor data and outputs position of the camera with respect to the middle of the markers pattern. 
   * Returns data raw, as well as filtered by a lowpass filter. 
   */
  virtual void run() {
    while (starting); 
    running = true;
    while (running) {
      pixy.ccc.getBlocks();
      nofBlocks = pixy.ccc.numBlocks;
      for(int i=0;i<nrDots; i++) {
        auto block = pixy.ccc.blocks[i];
        pos(i,0) = block.m_x;
        pos(i,1) = block.m_y;
      }
      double i_x_min = 0;
      double i_x_max = 0;
      double i_y_min = 0;
      double i_y_max = 0;
      double xO = 0;
      double yO = 0;

      for (int i=0; i<4; i++) {
        xO = xO + pos.get(i,0)/4;
        yO = yO + pos.get(i,1)/4;
        if (pos.get(i,0) < pos.get(i_x_min,0))
          i_x_min = i;
        if (pos.get(i,0) > pos.get(i_x_max,0))
          i_x_max = i;
        if (pos.get(i,1) < pos.get(i_y_min,1))
          i_y_min = i;
        if (pos.get(i,1) > pos.get(i_y_max,1))
          i_y_max = i;
      }
    
      // Shift centre ref system to middle
      xO = xO - max_pixel_x/2;
      yO = yO - max_pixel_y/2;

      double scaling_height = 3.02 / (3.02 + 1.43);
      double distWorld = 0.335 * scaling_height; 
      double distCam = (sqrt((pos.get(i_x_max,0) - pos.get(i_x_min,0)) * (pos.get(i_x_max,0) - pos.get(i_x_min,0))
            + (pos.get(i_x_max,1) - pos.get(i_x_min,1)) * (pos.get(i_x_max,1) - pos.get(i_x_min,1)) )
            + sqrt((pos.get(i_y_max,0) - pos.get(i_y_min,0)) * (pos.get(i_y_max,0) - pos.get(i_y_min,0))
            + (pos.get(i_y_max,1) - pos.get(i_y_min,1)) * (pos.get(i_y_max,1) - pos.get(i_y_min,1))))/2;
    
      // Define height from markers
      double pixel_fix = 93;     // pixel
      double height_fix = 0.915; // m
      height = pixel_fix * height_fix / distCam;
    
      // Define if markers in central region on picture
      if(pos.get(i_x_min,0) > delta_pixel_range && pos.get(i_x_min,0) < (max_pixel_x-delta_pixel_range) &&
      pos.get(i_x_max,0) > delta_pixel_range && pos.get(i_x_max,0) < (max_pixel_x-delta_pixel_range) &&
      pos.get(i_y_min,0) > delta_pixel_range && pos.get(i_y_min,0) < (max_pixel_x-delta_pixel_range) &&
      pos.get(i_y_max,0) > delta_pixel_range && pos.get(i_y_max,0) < (max_pixel_x-delta_pixel_range) &&
      pos.get(i_x_min,1) > delta_pixel_range && pos.get(i_x_min,1) < (max_pixel_y-delta_pixel_range) &&
      pos.get(i_x_max,1) > delta_pixel_range && pos.get(i_x_max,1) < (max_pixel_y-delta_pixel_range) &&
      pos.get(i_y_min,1) > delta_pixel_range && pos.get(i_y_min,1) < (max_pixel_y-delta_pixel_range) &&
      pos.get(i_y_max,1) > delta_pixel_range && pos.get(i_y_max,1) < (max_pixel_y-delta_pixel_range) )
        markersInRange = true;
      else
        markersInRange = false;
    
      if(height < height_limit && markersInRange == true) valid = true;
      else valid = false;
                  
      double xS = xO * distWorld / distCam;
      double yS = yO * distWorld / distCam;

      double phiS = (atan2((pos.get(i_x_max,1) - pos.get(i_x_min,1)), (pos.get(i_x_max,0) - pos.get(i_x_min,0)))
            + atan2((pos.get(i_y_min,0) - pos.get(i_y_max,0)), (pos.get(i_y_max,1) - pos.get(i_y_min,1)))) / 2;
      
      // Filtering
      double alpha_xy = 0.05; 
      double alpha_phi = 0.1; 
      double xS_filtered, yS_filtered, phiS_filtered;
    
      if(first){
        xS_filtered = xS;
        yS_filtered = yS;
        phiS_filtered = phiS;
        first = false;
      } else {
        xS_filtered = xS*alpha_xy + xS_prev*(1-alpha_xy);
        yS_filtered = yS*alpha_xy + yS_prev*(1-alpha_xy);
        phiS_filtered = phiS*alpha_phi + phiS_prev*(1-alpha_phi);
      }
      xS_prev = xS_filtered;
      yS_prev = yS_filtered;
      phiS_prev = phiS_filtered;

      double xS_out_f, yS_out_f;
      xS_out_f = yS_filtered;
      yS_out_f = xS_filtered;
        
      // Set output 
      outputRaw << xS, yS, phiS;
      output << xS_out_f, yS_out_f, phiS_filtered;
      
      usleep(5000);
    }
  }
  
  std::atomic<bool> starting;
  std::atomic<bool> running;
  bool first = true;
  bool valid = false;
  bool markersInRange = false;
  int nofBlocks;
  Matrix<nrDots,2,double> pos;
  Vector3 output, outputRaw;
  double height = 0;
  Pixy2 pixy;
  Logger log;
  double xS_prev = 0;
  double yS_prev = 0;
  double phiS_prev = 0;
  double max_pixel_x = 316;
  double max_pixel_y = 208;
  double delta_pixel_range = 10;
  double height_limit = 1.2;
};
  
}
}

#endif // ORG_EEROS_HAL_PIXYCAM_HPP_
