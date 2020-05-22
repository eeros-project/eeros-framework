#ifndef ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
#define ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen.h>
#include <canopen-drv.h>
#include <vector>
#include <algorithm>
#include <memory>

using namespace eeros::control;
using namespace eeros::logger;

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
 * @since v1.1
 */

class CanReceiveFaulhaber: public Block {
 public:
  CanReceiveFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode) 
      :  socket(socket), nodes(node), functionCodes(functionCode), log('C') {
    log.info() << "CAN receive block constructed, " << node.size() << " nodes with " << functionCode.size() << " PDO's";
    for (int i = 0; i < node.size(); i++) {
      outStatus.push_back(std::make_unique<Output<uint16_t>>(Output<uint16_t>()));
      outStatus[i]->getSignal().clear();
      outPos.push_back(std::make_unique<Output<double>>(Output<double>()));
      outPos[i]->getSignal().clear();
      outPosScale.push_back(1.0);
      outVel.push_back(std::make_unique<Output<double>>(Output<double>()));
      outVel[i]->getSignal().clear();
      outVelScale.push_back(1.0);
      outWarning.push_back(std::make_unique<Output<uint32_t>>(Output<uint32_t>()));
      outWarning[i]->getSignal().clear();
    }
  }
          
  virtual ~CanReceiveFaulhaber() { }
  
  /**
   * Copying of this object is forbidden due to the internal use of unique_ptr
   */
  CanReceiveFaulhaber(const CanReceiveFaulhaber&) = delete;

  virtual void run() {
    if (enabled) {
      uint32_t nof = nodes.size() * functionCodes.size();
      for (int i = 0; i < nof; i++) {
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
          if (it == nodes.end()) throw eeros::Fault("CAN node not found");
          int node = std::distance(nodes.begin(), it);
          if(readFrame.function_code == CANOPEN_FC_PDO1_TX) {
            uint16_t tmpStatus = (readFrame.payload.data[0] & 0xFF);
            tmpStatus |= (uint16_t(readFrame.payload.data[1] & 0xFF) << 8);
            outStatus[node]->getSignal().setValue(tmpStatus & 0x26F);
            outStatus[node]->getSignal().setTimestamp(ts);
            int32_t tmpVel = (readFrame.payload.data[2] & 0x00FF);
            tmpVel |= ((readFrame.payload.data[3] & 0x00FF) << 8);
            tmpVel |= ((readFrame.payload.data[4] & 0x00FF) << 16);
            tmpVel |= ((readFrame.payload.data[5] & 0x00FF) << 24);
            outVel[node]->getSignal().setValue(tmpVel * outVelScale[node]);
            outVel[node]->getSignal().setTimestamp(ts);
          } else if (readFrame.function_code == CANOPEN_FC_PDO2_TX) {
            int32_t tmpPos = (readFrame.payload.data[0] & 0x00FF);
            tmpPos |= ((readFrame.payload.data[1] & 0x00FF) << 8);
            tmpPos |= ((readFrame.payload.data[2] & 0x00FF) << 16);
            tmpPos |= ((readFrame.payload.data[3] & 0x00FF) << 24);
            outPos[node]->getSignal().setValue(tmpPos * outPosScale[node]);
            outPos[node]->getSignal().setTimestamp(ts);
            uint32_t tmpWarning = (readFrame.payload.data[4] & 0x00FF);
            tmpWarning |= ((readFrame.payload.data[5] & 0x00FF) << 8);
            tmpWarning |= ((readFrame.payload.data[6] & 0x00FF) << 16);
            tmpWarning |= ((readFrame.payload.data[7] & 0x00FF) << 24);
            outWarning[node]->getSignal().setValue(tmpWarning);
            outWarning[node]->getSignal().setTimestamp(ts);
          } else log.warn() << "PDO not parsed";
        }
      }
    }
  }

  virtual Output<uint16_t>& getOutStatus(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outStatus[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<double>& getOutPos(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outPos[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<double>& getOutVel(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outVel[i]);
      }
    }
    throw eeros::Fault("Error: specified CAN node not found");
  }
  
  virtual Output<uint32_t>& getOutWarning(uint8_t node) {
    for (int i = 0; i < nodes.size(); i++) {
      if (node == nodes[i]) {
        return (*outWarning[i]);
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
  
  virtual void setPosScale(uint8_t node, double scale) {
    outPosScale[node] = scale;
  }

  virtual void setVelScale(uint8_t node, double scale) {
    outVelScale[node] = scale;
  }


 private:
  int socket;
  bool enabled = false;
  Logger log;
  std::vector<std::unique_ptr<Output<uint16_t>>> outStatus;
  std::vector<std::unique_ptr<Output<double>>> outPos;
  std::vector<double> outPosScale;
  std::vector<std::unique_ptr<Output<double>>> outVel;
  std::vector<double> outVelScale;
  std::vector<std::unique_ptr<Output<uint32_t>>> outWarning;
  std::vector<uint8_t> nodes;
  std::vector<uint8_t> functionCodes;
};

}
}

#endif // ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
