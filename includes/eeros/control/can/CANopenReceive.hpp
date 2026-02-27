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
#include <atomic>
#include <initializer_list>
#include <cstring>

using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::logger;

namespace eeros {
namespace control {

/**
 * This block serves to receive CANopen messages from one or several drives implementing the DS402
 * specification. The drive has to be initialized and brought up to its operational state
 * through SDO transfers. After this, SDO transfer must stop. All further communication is
 * then done through PDO transfer. These PDOs must have been configured on the drive beforehand.
 *
 * The drive must be configured to send TPDOs which are received by this block.
 * You have to configure this block by setting all PDOs which should be received
 * by this block. They must be configured as TPDO (though they are sent by this block),
 * because the drive reads them as TPDO.
 * When configuring this TPDO, you have to indicate the node id of the drive, the TPDO number,
 * CANopen objects which will be packed into this PDO and a signal index. This index holds
 * information about which signal from the output of this block will be transmitted with this
 * PDO.
 *
 * The drive will send its TPDOs upen receiving a sync package. This must be
 * sent by a CANopenSend block.
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
   * @param co - CANopen object
   * @param node - vector with node id's of all connected CAN nodes 
   */
  template<typename CanOpen, typename NodeList = std::initializer_list<uint8_t>>
  CANopenReceive(CanOpen&& co, NodeList node)
      :  co(co), node(node), log(Logger::getLogger('C')) {
    for (size_t i = 0; i < node.size(); i++) {
      scale[i] = 1;
    }
    log.info() << "CAN receive block constructed with " << node.size() << " nodes";
    last_ts = eeros::System::getTimeNs();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  CANopenReceive(const CANopenReceive& s) = delete;

  /**
   * Reads from the CAN bus.
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() override {
    if (enabled.load(std::memory_order_relaxed)) {
      uint64_t ts = eeros::System::getTimeNs();
      for (uint32_t i = 0; i < nofPDO; i++) {
        coFrame_t frame;
        int err = co.frameRecv(frame);
        if (err == 0) {
          switch(frame.fnctCode) {
            case CANopen::FNCT_CODE::PDO1_TX:
            case CANopen::FNCT_CODE::PDO2_TX:
            case CANopen::FNCT_CODE::PDO3_TX:
            case CANopen::FNCT_CODE::PDO4_TX:
              handlePDO(frame);
              break;
            default:
              log.trace() << "unhandled CANOpen frame: fnctCode = 0x" << std::hex << (int)frame.fnctCode << ", id = 0x" << frame.id;
          }
        }
      }
    }
  }

  void handlePDO(coFrame_t& frame) {
          auto ts = eeros::System::getTimeNs();
          // search for registered PDOreceive
          for (const auto& p : pdo) {
            uint8_t n = std::get<1>(p);
            uint8_t r = std::get<2>(p);
            auto node = frame.id;
            auto TPDOnr = ((frame.fnctCode - CANopen::FNCT_CODE::PDO1_TX) >> 1) + 1;
            if (node == n && TPDOnr == r) {
              uint8_t nodeNr = std::get<0>(p);
              std::vector<uint8_t> length = std::get<3>(p);
              std::vector<int8_t> idx = std::get<4>(p);
              uint8_t i = 0, start = 0;
              for (auto& len : length) {
                len /= 8;
                int32_t val = co.getPDOdata(&frame.payload.data[0], start, start + len - 1);
                // log.info() << "got value " << val << " for node:index " << (int)nodeNr << ':' <<(int)idx[i];
                if (idx[i] > 0) {
                  auto sig = this->getOut(nodeNr).getSignal().getValue();
                  sig[idx[i]-1] = val * scale[nodeNr];
                  this->getOut(nodeNr).getSignal().setValue(sig);
                  this->getOut(nodeNr).getSignal().setTimestamp(ts);
                } else if (idx[i] < 0) {
                  auto sig = this->getDigOut(nodeNr).getSignal().getValue();
                  sig[-idx[i]-1] = val;
                  this->getDigOut(nodeNr).getSignal().setValue(sig);
                  this->getDigOut(nodeNr).getSignal().setTimestamp(ts);
                } else {/*std::cout << "R:status[" << (int)nodeNr << "] = 0x" << std::hex << val << '\n';*/ status[nodeNr] = val;}// & 0x26F;
                start += len;
                i++;
              }
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
  virtual void enable() override {
    nofPDO = pdo.size();
    enabled.store(true, std::memory_order_relaxed);
    log.trace() << "enabling can receive block";
  }
  
  /**
   * Disables the block.
   *
   * If disabled, no PDOs will be read.
   *
   * @see run()
   */
  virtual void disable() override {
    enabled.store(false, std::memory_order_relaxed);
    log.trace() << "disabling can receive block";
  }
  
  /**
   * Sets the scaling.
   *
   * The drive delivers its position and velocity information as 4 bytes counter values.
   * The scaling allows to transform this counter values into meaningful position and
   * velocity information in rad, m, rad/s or m/s.
   *
   * @param scale the scaling factor for the velocity for all drives
   */
  virtual void setScale(Matrix<N,M,double>& scale) {
    this->scale = scale;
  }

  /**
   * Reads the status information.
   *
   * @param index index of the drive
   */
  virtual uint16_t getStatus(uint8_t index) {
    return this->status[index];
  }

  /**
   * Configures the PDO which are received by this block.
   *
   * First of all this includes the node id of the slave drive. This id is searched for in the
   * list with all node ids and the corresponding index is saved as well for later use.
   * Next, you have to specify which TPDO number is chosen together with one or several
   * CANopen dictionary objects which are mapped into this TPDO. Last, you must chose for
   * each mapping, where the corresponding signal goes to.
   *
   * @param nodeId the id of the node to which this PDO is sent
   * @param TPDOnr nr of the RPDO (1 to 4)
   * @param objs opjects which are mapped into this PDO
   * @param idx signal index, where
   *            - > 0 : index in floating point inputs
   *            - 0   : control word
   *            - < 0 : index in integer inputs
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
  CANopen co;
  std::atomic<bool> enabled = false;
  Output<Matrix<P,1,uint32_t>> digOut[N];
  Matrix<N,1,uint16_t> status;
  Matrix<N,M,double> scale;
  std::vector<uint8_t> node;
  // node index, node id, RPDOnr, length in bit of all objects, signal index of all objects
  std::vector<std::tuple<uint8_t,uint8_t,uint8_t,std::vector<uint8_t>,std::vector<int8_t>>> pdo;
  uint32_t nofPDO;
  Logger log;
  uint64_t last_ts = 0;
};

}
}

#endif // ORG_EEROS_CONTROL_CANOPENRECEIVE_
