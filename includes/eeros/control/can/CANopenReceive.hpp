#ifndef ORG_EEROS_CONTROL_CANOPENRECEIVE_
#define ORG_EEROS_CONTROL_CANOPENRECEIVE_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <CANopen.hpp>
#include <vector>
#include <algorithm>
#include <initializer_list>


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
 * @tparam M - each drive can have several floating point output signals (1 - default)
 *             e.g. velocity and position
 * @tparam P - each drive can have several integer output signals (1 - default)
 *             e.g. status word and digital inputs
 *
 * @since v1.2
 */
template < uint8_t N = 2, uint8_t M = 1, uint8_t P = 1 >
class CANopenReceive : public Blockio<0,N,Matrix<M,1,double>> {
  
 public:
  /**
   * Constructs a CAN receive block instance for a given set of nodes.
   * Sets the scale of velocity and position to 1.
   *
   * @param socket - socket of number of associated CAN bus
   * @param node - vector with node id's of all connected CAN nodes 
   * @param functionCode - vector with function codes of all PDO's to be received
   */
  CANopenReceive(CANopen& co, std::initializer_list<uint8_t> node)
      :  co(co), node(node), log(Logger::getLogger('C')) {
    for (size_t i = 0; i < node.size(); i++) {
      scale[i] = 1;
    }
    log.info() << "CAN receive block constructed with " << node.size() << " nodes";
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  CANopenReceive(const CANopenReceive& s) = delete;

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
      uint64_t ts = eeros::System::getTimeNs();
      for (uint32_t i = 0; i < nofPDO; i++) {
        // read PDO
        uint8_t node, TPDOnr, buf[8], len;
        int err;
        if ((err = co.PDOreceive(node, TPDOnr, buf, len)) == 0) {
//           std::cout << "PDO received from: node=" << (int)node << " TPDOnr=" << (int)TPDOnr << " len=" << (int)len
//           << " status=0x" << std::hex << co.getPDOdata(buf, 0, 1)
//           << " pos=0x" << co.getPDOdata(buf, 2, 5) << std::endl;
          // search for registered PDOreceive
          for (const auto& p : pdo) {
            uint8_t n = std::get<1>(p);
//             log.error() << (int)n << " " << (int)node;
            uint8_t r = std::get<2>(p);
            if (node == n && TPDOnr == r) {
              uint8_t nodeNr = std::get<0>(p);
              std::vector<uint8_t> length = std::get<3>(p);
              std::vector<int8_t> idx = std::get<4>(p);
              uint8_t i = 0, start = 0;
              for (auto& len : length) {
                len /= 8;
                int32_t val = co.getPDOdata(buf, start, start + len - 1);
                if (idx[i] > 0) {
//                   log.fatal() << (int)nodeNr << " " << (int)n << " " << (int)idx[i];
                  auto sig = this->getOut(nodeNr).getSignal().getValue();
                  sig[idx[i]-1] = val;
                  this->getOut(nodeNr).getSignal().setValue(sig);
                  this->getOut(nodeNr).getSignal().setTimestamp(ts);
                } else if (idx[i] < 0) {
                  auto sig = this->getDigOut(nodeNr).getSignal().getValue();
                  sig[-idx[i]-1] = val;
                  this->getDigOut(nodeNr).getSignal().setValue(sig);
                  this->getDigOut(nodeNr).getSignal().setTimestamp(ts);
                } else status[nodeNr] = val & 0x26F;
                start += len;
                i++;
              }
            }
          }

//           uint16_t st = co.getPDOdata(buf, 0, 1);
//           if ((st & 0x417f) != 0x137) {
//             co.sendNMT(0, co.NMT_CS::PREOP);
// //             std::cout << "status=" << ds402.getStatusDesc(node) << std::endl;
// //             std::cout << "error=" << ds402.getErrorDesc(node) << std::endl;
//             log.error() << "status wrong";
//             break;
//           }

        } else log.warn() << err;
      }
    }
  }
  
  /**
   * Enables the block.
   *
   * If enabled, run() will read PDOs on the CAN bus and parse them.
   *
   * @see run()
   */
  virtual void enable() {
    nofPDO = pdo.size();
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
   * Sets the scaling for the velocity information.
   *
   * The drive needs its velocity information as a 4 bytes counter value.
   * The scaling allows to transform this counter value into meaningful
   * velocity information in rad/s or m/s.
   *
   * @param scale the scaling factor for the velocity for all drives
   */
  virtual void setScale(Matrix<N,M,double>& scale) {
    this->scale = scale;
  }

  /**
   * Sets the scaling for the velocity information.
   *
   * The drive needs its velocity information as a 4 bytes counter value.
   * The scaling allows to transform this counter value into meaningful
   * velocity information in rad/s or m/s.
   *
   * @param scale the scaling factor for the velocity for all drives
   */
  virtual uint16_t getStatus(uint8_t index) {
    return this->status[index];
  }

  /**
   * Sets the scaling for the velocity information.
   *
   * The drive needs its velocity information as a 4 bytes counter value.
   * The scaling allows to transform this counter value into meaningful
   * velocity information in rad/s or m/s.
   *
   * @param nodeId the id of the node for which this PDO is sent
   * @param TPDOnr nr of the TPDO (1 to 4)
   * @param objs The scaling factor for the velocity for all drives
   * @param idx signal index
   */
  virtual void configureTPDO(uint8_t nodeId, uint8_t TPDOnr, std::vector<coObject_t> objs, std::vector<int8_t> idx) {
    std::vector<uint8_t>::iterator it = std::find(node.begin(), node.end(), nodeId);
    if (it == node.end()) {
      log.warn() << "CAN configure TPDO: node id " << nodeId << " not found";
      return;
    }
    uint8_t nodeIdx = std::distance(node.begin(), it);
    std::vector<uint8_t> len;
    for (coObject_t o : objs) {
      len.push_back(o.len);
    }
    pdo.push_back(make_tuple(nodeIdx, nodeId, TPDOnr, len, idx));
  }

  /**
   * Get the output of the digital signals of the block.
   *
   * @param index - index of the output
   * @return output
   */
  virtual Output<Matrix<P,1,uint32_t>>& getDigOut(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
    return digOut[index];
  }

 private:
  CANopen& co;
  bool enabled = false;
  Output<Matrix<P,1,uint32_t>> digOut[N];
  Matrix<N,1,uint16_t> status;
  Matrix<N,M,double> scale;
  std::vector<uint8_t> node;
  // node index, node id, RPDOnr, length in bit of all objects, signal index of all objects
  std::vector<std::tuple<uint8_t,uint8_t,uint8_t,std::vector<uint8_t>,std::vector<int8_t>>> pdo;
  uint32_t nofPDO;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANOPENRECEIVE_
