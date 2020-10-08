#ifndef ORG_EEROS_CONTROL_CANSENDFAULHABER_
#define ORG_EEROS_CONTROL_CANSENDFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen-com.h>

using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::math;

namespace eeros {
namespace control {

/**
 * This block serves to send CAN messages to a Faulhaber or similar drive.
 * The drive has to be initialized and brought up to its operational state 
 * through SDO transfers. After this, SDO transfer must stop. All further 
 * communication is done through PDO transfer. These PDOs must have been 
 * configured on the drive beforehand. 
 * 
 * The drive must be configured to receive RPDOs as follows
 *   RPDO1: control word (16 bit unsigned)
 *   RPDO2: IPrec1 (32 bit signed), IPrec2 (32 bit signed)
 *
 * When enabled this block will transmit a sync package each time it runs.

 * @tparam N - number of CAN nodes (2 - default)
 *
 * @since v1.2
 */
template < uint8_t N = 2 >
class CanSendFaulhaber : public Block {
  
 public:
  /**
   * Constructs a CAN send block instance for a given set of nodes.
   * Sets the scale of velocity and position to 1.
   *
   * @param socket - socket of number of associated CAN bus
   * @param node - vector with node id's of all connected CAN nodes 
   * @param functionCode - vector with function codes of all PDO's to be received
   */
  CanSendFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode)
      : socket(socket), nodes(node), functionCodes(functionCode), log('Y') {
    for (size_t i = 0; i < node.size(); i++) {
      velScale[i] = 1;
    }
    lastCtrl = 0;
    ctrl = 0;
    log.info() << "CAN send block constructed, " << node.size() << " nodes with " << functionCode.size() << " PDO's";
  }

  /**
   * Transmits on the CAN bus. If enabled a sync package is sent for each run. After this the block
   * will send two RPDO's to each of the connected CAN nodes.
   * The RPDO2 will only be sent if the drive is set to interpolated position mode.
   *
   * If enabled transmits 2 PDO's to each CAN node. 
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() {
    if (enabled) {
      int err;
      if ((err = canopen_send_sync(socket)) != 0) throw eeros::Fault("send sync failed");
        
      // send control word to all nodes
      for (uint32_t i = 0; i < nodes.size(); i++) {  
        if (ctrl[i] != lastCtrl[i]) {
          log.info() << "CAN ctrl changed for node " << std::to_string(nodes[i]) << ": send 0x" << std::hex << ctrl[i];
          err = canopen_pdo_send_2bytes(socket, nodes[i], CANOPEN_FC_PDO1_RX, ctrl[i]);
          lastCtrl[i] = ctrl[i];
          if (err != 0) throw eeros::Fault(std::string("CAN send ctrl to node ") + std::to_string(nodes[i]) + " failed");
        }
      }
        
      // send velocity reference to all nodes      
      if (ipMode) {
        for (std::size_t i = 0; i < nodes.size(); i++) {
          err = canopen_pdo_send_4bytes(socket, nodes[i], CANOPEN_FC_PDO2_RX, vel.getSignal().getValue()[i] * velScale[i]);
          if (err != 0) throw eeros::Fault("set of velocity over CAN failed");
        }
      }
    }
  }
        
  /**
   * Getter function for the velocity input.
   * 
   * @return The input carrying the velocity information
   */
  virtual Input<Matrix<N,1,double>>& getInVel() {
    return vel;
  }
          
  /**
   * Setter function for the control word.
   * 
   * @param ctrl The control word information for all drives
   */
  virtual void setCtrl(Matrix<N,1,uint16_t>& ctrl) {
    this->ctrl = ctrl;
  }

  /**
   * Enables the block.
   *
   * If enabled, run() will send a sync package followed by PDOs onto the CAN bus.
   *
   * @see run()
   */
  virtual void enable() {
    enabled = true;
  }
  
  /**
   * Disables the block.
   *
   * If disabled, no sync and PDOs will be sent.
   *
   * @see run()
   */
  virtual void disable() {
    enabled = false;
  }

  /**
   * Enables the interpolated position mode.
   *
   * If enabled, run() will send the RPDO2 package.
   *
   * @see run()
   */
  virtual void ipmodeEnable() {
    ipMode = true;
  }
  
  /**
   * Disables the interpolated position mode.
   *
   * If disabled, run() will not send the RPDO2 package.
   *
   * @see run()
   */
  virtual void ipmodeDisable() {
    ipMode = false;
  }

  /**
   * Sets the scaling for the velocity information.
   *
   * The drive needs its velocity information as a 4 bytes counter value.
   * The scaling allows to transform this counter value into meaningful 
   * velocity information in rad/s or m/s.
   *
   * @param scale The scaling factor for the velocity for all drives
   */
  virtual void setVelScale(Matrix<N,1,double>& scale) {
    velScale = scale;
  }

 private:
  int socket;
  bool enabled = false;
  bool ipMode = false;
  Input<Matrix<N,1,double>> vel;
  Matrix<N,1,double> velScale;
  Matrix<N,1,uint16_t> ctrl;
  Matrix<N,1,uint16_t> lastCtrl;
  std::vector<uint8_t> nodes;
  std::vector<uint8_t> functionCodes;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANSENDFAULHABER_
