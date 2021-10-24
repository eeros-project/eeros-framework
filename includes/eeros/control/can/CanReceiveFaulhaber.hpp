#ifndef ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
#define ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen-drv.h>
#include <vector>
#include <algorithm>


using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::logger;

namespace eeros {
namespace control {

/**
 * This block serves to receive CAN messages from a Faulhaber or similar drive.
 * The drive has to be initialized and brought up to its operational state 
 * through SDO transfers. After this, SDO transfer must stop. All further 
 * communication is done through PDO transfer. These PDOs must have been 
 * configured on the drive beforehand. 
 * 
 * The drive must be configured to send TPDOs as follows
 *   TPDO1: status word (16 bit unsigned), actual velocity (32 bit signed)
 *   TPDO2: actual position (32 bit signed), warnings (32 bit unsigned)
 *
 * The drive will send its TPDOs upen receiving a sync package. This must be
 * sent by a CanSend block.
 *
 * @tparam N - number of CAN nodes (2 - default)
 *
 * @since v1.2
 */
template < uint8_t N = 2 >
class CanReceiveFaulhaber: public Block {
  
 public:
  /**
   * Constructs a CAN receive block instance for a given set of nodes.
   * Sets the scale of velocity and position to 1.
   *
   * @param socket - socket of number of associated CAN bus
   * @param node - vector with node id's of all connected CAN nodes 
   * @param functionCode - vector with function codes of all PDO's to be received
   */
  CanReceiveFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode) 
      :  socket(socket), nodes(node), functionCodes(functionCode), log(Logger::getLogger('C')) {
    for (size_t i = 0; i < node.size(); i++) {
      posScale[i] = 1;
      velScale[i] = 1;
    }
    log.info() << "CAN receive block constructed, " << node.size() << " nodes with " << functionCode.size() << " PDO's";
  }
          
  /**
   * Reads from the CAN bus.
   *
   * If enabled reads 2 PDO's from each CAN node. 
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() {
    if (enabled) {
      uint32_t nof = nodes.size() * functionCodes.size();
      for (uint32_t i = 0; i < nof; i++) {
        uint64_t ts = eeros::System::getTimeNs();
        int readLen = 0;
        canopen_frame_t readFrame;
        readLen = canopen_read_frame(socket, &readFrame);
        if (readLen == EAGAIN || readLen == EWOULDBLOCK){
          log.error() << "would block";
        }
        if (readLen < 0) {
          if(readLen != -1) log.info() << "error " << readLen;
        }
        if (readLen > 0) {
          // find associated node
          std::vector<uint8_t>::iterator it = std::find(nodes.begin(), nodes.end(), readFrame.id);
          if (it == nodes.end()) log.warn() << "CAN receive: node id " << readFrame.id << " not found";
          int node = std::distance(nodes.begin(), it);
          if(readFrame.function_code == CANOPEN_FC_PDO1_TX) {
            uint16_t tmpStatus = (readFrame.payload.data[0] & 0xFF);
            tmpStatus |= (uint16_t(readFrame.payload.data[1] & 0xFF) << 8);
            auto s = status.getSignal().getValue();
            s[node] = tmpStatus & 0x26F;
            status.getSignal().setValue(s);
            status.getSignal().setTimestamp(ts);
            int32_t tmpVel = (readFrame.payload.data[2] & 0x00FF);
            tmpVel |= ((readFrame.payload.data[3] & 0x00FF) << 8);
            tmpVel |= ((readFrame.payload.data[4] & 0x00FF) << 16);
            tmpVel |= ((readFrame.payload.data[5] & 0x00FF) << 24);
            auto v = vel.getSignal().getValue();
            v[node] = tmpVel / velScale[node]; 
            vel.getSignal().setValue(v);
            vel.getSignal().setTimestamp(ts);
          } else if (readFrame.function_code == CANOPEN_FC_PDO2_TX) {
            int32_t tmpPos = (readFrame.payload.data[0] & 0x00FF);
            tmpPos |= ((readFrame.payload.data[1] & 0x00FF) << 8);
            tmpPos |= ((readFrame.payload.data[2] & 0x00FF) << 16);
            tmpPos |= ((readFrame.payload.data[3] & 0x00FF) << 24);
            auto p = pos.getSignal().getValue();
            p[node] = tmpPos / posScale[node]; 
            pos.getSignal().setValue(p);
            pos.getSignal().setTimestamp(ts);
            uint32_t tmpWarning = (readFrame.payload.data[4] & 0x00FF);
            tmpWarning |= ((readFrame.payload.data[5] & 0x00FF) << 8);
            tmpWarning |= ((readFrame.payload.data[6] & 0x00FF) << 16);
            tmpWarning |= ((readFrame.payload.data[7] & 0x00FF) << 24);
            auto w = warning.getSignal().getValue();
            w[node] = tmpWarning; 
            warning.getSignal().setValue(w);
            warning.getSignal().setTimestamp(ts);
          } else log.warn() << "PDO not parsed: " << (int)readFrame.function_code;
        }
      }
    }
  }

  /**
   * Getter function for the status output.
   * 
   * @return The output carrying the status information
   */
  virtual Output<Matrix<N,1,uint16_t>>& getOutStatus() {
        return status;
  }
  
  /**
   * Getter function for the position output.
   * 
   * @return The output carrying the position information
   */
  virtual Output<Matrix<N,1,double>>& getOutPos() {
    return pos;
  }
  
  /**
   * Getter function for the velocity output.
   * 
   * @return The output carrying the velocity information
   */
  virtual Output<Matrix<N,1,double>>& getOutVel() {
    return vel;
  }
  
  /**
   * Getter function for the warning output.
   * 
   * @return The output carrying the warning information
   */
  virtual Output<Matrix<N,1,uint32_t>>& getOutWarning() {
    return warning;
  }

  /**
   * Enables the block.
   *
   * If enabled, run() will read PDOs on the CAN bus and parse them.
   *
   * @see run()
   */
  virtual void enable() {
    enabled = true;
  }
  
  /**
   * Disables the block.
   *
   * If disabled, no PDOs will be read.
   *
   * @see run()
   */
  virtual void disable() {
    enabled = false;
  }
  
  /**
   * Sets the scaling for the position information.
   *
   * The drive sends its positioning information as a 4 bytes counter value.
   * The scaling allows to transform this counter value into meaningful 
   * position information in rad or m.
   *
   * @param scale The scaling factor for the position for all drives
   */
  virtual void setPosScale(Matrix<N,1,double>& scale) {
    posScale = scale;
  }

  /**
   * Sets the scaling for the velocity information.
   *
   * The drive sends its velocity information as a 4 bytes counter value.
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
  Output<Matrix<N,1,uint16_t>> status;
  Output<Matrix<N,1,double>> pos;
  Matrix<N,1,double> posScale;
  Output<Matrix<N,1,double>> vel;
  Matrix<N,1,double> velScale;
  Output<Matrix<N,1,uint32_t>> warning;
  std::vector<uint8_t> nodes;
  std::vector<uint8_t> functionCodes;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
