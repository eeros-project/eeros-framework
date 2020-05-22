#ifndef ORG_EEROS_CONTROL_CANSENDFAULHABER_
#define ORG_EEROS_CONTROL_CANSENDFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen-drv.h>
#include <canopen.h>
#include <canopen-com.h>

using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::math;

namespace eeros {
namespace control {

template < uint8_t N = 2 >
class CanSendFaulhaber : public Block {
 public:
  CanSendFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode)
      : socket(socket), nodes(node), functionCodes(functionCode), log('Y') {
    log.info() << "CAN send block constructed, " << node.size() << " nodes with " << functionCode.size() << " PDO's";
  }

  virtual ~CanSendFaulhaber() {
    nodes.clear();
  }

  virtual void run() {
    if (enabled) {
      int err;
      if ((err = canopen_send_sync(socket)) != 0) throw eeros::Fault("send sync failed");
        
      //send control word to all nodes
      for (uint32_t i = 0; i < nodes.size(); i++) {  
        if (ctrl.getSignal().getValue()[i] != lastCtrl[i]) {
          log.info() << "CAN ctrl changed: send " << ctrl.getSignal().getValue()[i];
          err = canopen_pdo_send_2bytes(socket, nodes[i], CANOPEN_FC_PDO1_RX, ctrl.getSignal().getValue()[i]);
          lastCtrl[i] = ctrl.getSignal().getValue()[i];
          if (err != 0) throw eeros::Fault("CAN send ctrl failed");
        }
      }
        
      // send velocity reference to all nodes      
      if (ipMode) {
        for (std::size_t i = 0; i < nodes.size(); i++) {
          err = canopen_pdo_send_4bytes(socket, nodes[i], CANOPEN_FC_PDO2_RX, vel.getSignal().getValue()[i] * velScale[i]);
          if (err != 0) throw eeros::Fault("set of velocity over CAN failed");
        }
      }
        
//      if (ip_mode_enabled) {
//          for (int i = 0; i < nodes.size(); i++) {
//              err = 0;
//                  err = canopen_pdo_send_8bytes(socket, nodes[i], CANOPEN_FC_PDO2_RX, inPos[i]->getSignal().getValue(), inVel[i]->getSignal().getValue());
//              err = canopen_pdo_send_4bytes(socket, nodes[i], CANOPEN_FC_PDO2_RX, inVel[i]->getSignal().getValue());
//              
//                  if (err != 0) throw eeros::Fault("set of position over CAN failed");
//              if (err != 0) throw eeros::Fault("set of velocity over CAN failed");
//          }
//      }
    }
  }
        
  /**
   * Getter function for the input with a given index.
   * 
   * @tparam index - index of input
   * @return The input with this index
   */
  virtual Input<Matrix<N,1,double>>& getInVel() {
    return vel;
  }
          
  virtual Input<Matrix<N,1,uint16_t>>& getInCtrl() {
    return ctrl;
  }

  virtual void enable() {
    enabled = true;
  }
  
  virtual void disable() {
    enabled = false;
  }

  virtual void ipmodeEnable() {
    ipMode = true;
  }
  
  virtual void ipmodeDisable() {
    ipMode = false;
  }

  virtual void setVelScale(uint8_t node, double scale) {
    velScale[node] = scale;
  }

 private:
  int socket;
  bool enabled = false;
  bool ipMode = false;
  Input<Matrix<N,1,double>> vel;
  Matrix<N,1,double> velScale;
  Input<Matrix<N,1,uint16_t>> ctrl;
  Matrix<N,1,uint16_t> lastCtrl;
  std::vector<uint8_t> nodes;
  std::vector<uint8_t> functionCodes;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANSENDFAULHABER_