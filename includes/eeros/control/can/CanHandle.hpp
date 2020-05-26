#ifndef ORG_EEROS_CONTROL_CANHANDLE_HPP_
#define ORG_EEROS_CONTROL_CANHANDLE_HPP_

#include <eeros/core/Fault.hpp>
#include <can-if.h>
#include <canopen-drv.h>
#include <canopen-com.h>
#include <canopen-faulhaber-drv.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>

namespace eeros {
namespace control {

/**
 * A gain block is used to amplify an input signal. This is basically done by
 * multiplying the gain with the input signal value.
 * The following term represents the operation performed in this block.
 *
 * output = gain * input
 *
 * Gain is a class template with two type and one non-type template arguments.
 * The two type template arguments specify the types which are used for the
 * output type and the gain type when the class template is instanciated.
 * The non-type template argument specifies if the multiplication will be done
 * element wise in case the gain is used with matrices.
 *
 * A gain block is suitable for use with multiple threads. However,
 * enabling/disabling of the gain and the smooth change feature
 * is not synchronized.
 *
 * @tparam Tout - output type (double - default type)
 * @tparam Tgain - gain type (double - default type)
 * @tparam elementWise - amplify element wise (false - default value)
 *
 * @since v0.6
 */
class CanHandle {
 
 public:
  /**
   * Constructs a default gain instance with a gain of 1.\n
   * Calls Gain(Tgain c).
   *
   * @see Gain(Tgain c)
   */
  CanHandle(std::string iface) {
    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      throw eeros::Fault("Error: Failed to create can socket.");
      return;
    }
     
    /* Locate the interface you wish to use */
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(struct ifreq));
    iface.copy(ifr.ifr_name, iface.length(), 0);
//     if(ioctl(sock, SIOCGIFINDEX, &ifr) < 0){        // ifr.ifr_ifindex gets filled with that device's index
//       throw eeros::Fault("Error: Failed to locate can interface");
//       return;
//     }
    
    // Select that CAN interface, and bind the socket to it
//     struct sockaddr_can addr;
//     memset(&addr, 0, sizeof(struct sockaddr_can));
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;
//     if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0){
//         throw eeros::Fault("Error: Failed to bind can socket");
//         return;
//     }
//     int flags = fcntl(sock, F_GETFL, 0);
//     fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  }
                
  virtual ~CanHandle() {
    close(sock);
  }
  
  int getSocket() {
    return sock;
  }
                    
 private:
  int sock;
                
};
}
}

#endif // ORG_EEROS_CONTROL_CANHANDLE_HPP_

