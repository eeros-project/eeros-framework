#ifndef ORG_EEROS_CONTROL_CANHANDLE_HPP_
#define ORG_EEROS_CONTROL_CANHANDLE_HPP_

#include <eeros/core/Fault.hpp>
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
 * A CAN handle block is used create a socket connection to an
 * existing CAN device. The connection will be configured as nonblocking.
 * CanSend and CanReceive blocks will both need this block to get a 
 * handle to its socket connection.
 *
 * @since v1.2
 */
class CanHandle {
 
 public:
  /**
   * Constructs a handle to the socket file descriptor and opens the socket.\n
   *
   * @param iface - interface name
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
    if(ioctl(sock, SIOCGIFINDEX, &ifr) < 0){        // ifr.ifr_ifindex gets filled with that device's index
      throw eeros::Fault("Error: Failed to locate can interface");
      return;
    }
    
    // Select that CAN interface, and bind the socket to it
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(struct sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      throw eeros::Fault("Error: Failed to bind can socket");
      return;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }
                
  /**
   * Destructor.\n
   * Closes its socket.
   */
  virtual ~CanHandle() {
    close(sock);
  }
  
  /**
   * Getter function for the socket file descriptor.
   * 
   * @return The file descriptor of the CAN socket
   */
  int getSocket() {
    return sock;
  }
                    
 private:
  int sock;
                
};
}
}

#endif // ORG_EEROS_CONTROL_CANHANDLE_HPP_

