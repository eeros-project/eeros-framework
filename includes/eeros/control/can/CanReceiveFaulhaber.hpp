#ifndef ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_
#define ORG_EEROS_CONTROL_CANRECEIVEFAULHABER_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <canopen.h>
#include <canopen-drv.h>
#include <vector>
#include <algorithm>
#include <memory>

using namespace eeros::control;
using namespace eeros::math;
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
template < uint8_t N = 2 >
class CanReceiveFaulhaber: public Block {
 public:
  CanReceiveFaulhaber(int socket, std::initializer_list<uint8_t> node, std::initializer_list<uint8_t> functionCode) 
      :  socket(socket), nodes(node), functionCodes(functionCode), log('C') {
    log.info() << "CAN receive block constructed, " << node.size() << " nodes with " << functionCode.size() << " PDO's";
  }
          
  virtual ~CanReceiveFaulhaber() { }
  
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
          if (it == nodes.end()) throw eeros::Fault("CAN node not found");
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
            v[node] = tmpVel * velScale[node]; 
            vel.getSignal().setValue(v);
            vel.getSignal().setTimestamp(ts);
          } else if (readFrame.function_code == CANOPEN_FC_PDO2_TX) {
            int32_t tmpPos = (readFrame.payload.data[0] & 0x00FF);
            tmpPos |= ((readFrame.payload.data[1] & 0x00FF) << 8);
            tmpPos |= ((readFrame.payload.data[2] & 0x00FF) << 16);
            tmpPos |= ((readFrame.payload.data[3] & 0x00FF) << 24);
            auto p = pos.getSignal().getValue();
            p[node] = tmpPos * posScale[node]; 
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
          } else log.warn() << "PDO not parsed";
        }
      }
    }
  }

  virtual Output<Matrix<N,1,uint16_t>>& getOutStatus() {
        return status;
  }
  
  virtual Output<Matrix<N,1,double>>& getOutPos() {
    return pos;
  }
  
  virtual Output<Matrix<N,1,double>>& getOutVel() {
    return vel;
  }
  
  virtual Output<Matrix<N,1,uint32_t>>& getOutWarning() {
    return warning;
  }

  virtual void enable() {
    enabled = true;
  }
  
  virtual void disable() {
    enabled = false;
  }
  
  virtual void setPosScale(uint8_t node, double scale) {
    posScale[node] = scale;
  }

  virtual void setVelScale(uint8_t node, double scale) {
    velScale[node] = scale;
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
