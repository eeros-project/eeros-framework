#ifndef ORG_EEROS_CONTROL_CANSENDFAULHABER_
#define ORG_EEROS_CONTROL_CANSENDFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen-drv.h>
#include <canopen.h>
#include <canopen-com.h>

using namespace eeros::control;
using namespace eeros::logger;

namespace eeros {
namespace control {

class CanSendFaulhaber : public Block {
 public:
  CanSendFaulhaber(int socket, std::initializer_list<int> node)
      : socket(socket), nodes(node), lastCtrlValue(node.size(), 0), log('Y') {
    for (int i = 0; i < node.size(); i++) {
      inPos.push_back(new Input<double>());
      inPosScale.push_back(1.0);
      inVel.push_back(new Input<double>());
      inVelScale.push_back(1.0);
      inCtrl.push_back(new Input<uint16_t>());
      lastCtrlValue.push_back(0);
    }
  }

  virtual ~CanSendFaulhaber() {
    nodes.clear();
  }

  virtual void run() {
    if (enabled) {
      int err;
      if ((err = canopen_send_sync(socket)) != 0) throw eeros::Fault("send sync failed");
        
      //send control word to all nodes
      for (int i = 0; i < nodes.size(); i++) {  
        if (inCtrl[i]->getSignal().getValue() != lastCtrlValue[i]) {
          log.info() << "CAN ctrl changed: send " << inCtrl[i]->getSignal().getValue();
          err = canopen_pdo_send_2bytes(socket, nodes[i], CANOPEN_FC_PDO1_RX, inCtrl[i]->getSignal().getValue());
          lastCtrlValue[i] = inCtrl[i]->getSignal().getValue();
          if (err != 0) throw eeros::Fault("CAN send ctrl failed");
        }
      }
        
      //send velocity reference to all nodes      
      if (ipMode) {
        for (int i = 0; i < nodes.size(); i++) {
        err = canopen_pdo_send_4bytes(socket, nodes[i], CANOPEN_FC_PDO2_RX, inVel[i]->getSignal().getValue() * inVelScale[i]);
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
        
  Input<double>* getInVel(int node) {
    for (int i = 0; i < nodes.size(); i++) {
        if (nodes[i] == node) {
            return inVel[i];
        }
    }   
    throw eeros::Fault("Error: specified CAN node not found");
  }
          
  Input<uint16_t>* getInCtrl(int node) {
    for (int i = 0; i < nodes.size(); i++) {
        if (nodes[i] == node) {
            return inCtrl[i];
        }
    }   
    throw eeros::Fault("Error: specified CAN node not found");
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

  virtual void setPosScale(uint8_t node, double scale) {
    inPosScale[node] = scale;
  }

  virtual void setVelScale(uint8_t node, double scale) {
    inVelScale[node] = scale;
  }

 private:
  virtual void sendPdo2Bytes(int node, uint8_t function_code, uint16_t userData) {
    int err = canopen_pdo_send_2bytes(socket, node, function_code, userData);
    if (err != 0) throw eeros::Fault("send pdo failed");
  }
  virtual void sendPdo4Bytes(int node, uint8_t function_code, uint32_t userData) {
    int err = canopen_pdo_send_4bytes(socket, node, function_code, userData);
    if (err != 0) throw eeros::Fault("send pdo failed");
  }
  virtual void sendPdo6Bytes(int node, uint8_t function_code, uint16_t ctrlWord, uint32_t userData) {
    int err = canopen_pdo_send_6bytes(socket, node, function_code, ctrlWord, userData);
    if (err != 0) throw eeros::Fault("send pdo failed");
  }
  virtual void sendPdo8Bytes(int node, uint8_t function_code, uint32_t userData1, uint32_t userData2) {
    int err = canopen_pdo_send_8bytes(socket, node, function_code, userData1, userData2);
    if (err != 0) throw eeros::Fault("send pdo failed");
  }
		
  int socket;
  bool enabled = false;
  bool ipMode = false;
  Logger log;
  std::vector<Input<double>*> inPos;
  std::vector<double> inPosScale;
  std::vector<Input<double>*> inVel;
  std::vector<double> inVelScale;
  std::vector<Input<uint16_t>*> inCtrl;
  std::vector<uint16_t> lastCtrlValue;
  std::vector<int> nodes;
};

}
}

#endif // ORG_EEROS_CONTROL_CANSENDFAULHABER_