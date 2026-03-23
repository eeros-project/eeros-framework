#ifndef ORG_EEROS_CONTROL_CANOPENSEND_
#define ORG_EEROS_CONTROL_CANOPENSEND_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <CANopen.hpp>
#include <vector>
#include <initializer_list>
#include <algorithm>
#include <sstream>
#include <cstring>

using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::math;

namespace eeros {
namespace control {

/**
 * This block serves to send CANopen messages to one or several drives implementing the DS402
 * specification. The drive has to be initialized and brought up to its operational state
 * through SDO transfers. After this, SDO transfer must stop. All further communication is
 * then done through PDO transfer. These PDOs must have been configured on the drive beforehand.
 * 
 * The drive must be configured to receive RPDOs which are sent by this block.
 * You have to configure this block by setting all PDOs which should be transmitted
 * by this block. They must be configured as RPDO (though they are sent by this block),
 * because the drive reads them as RPDO.
 * When configuring this RPDO, you have to indicate the node id of the drive, the RPDO number,
 * CANopen objects which will be packed into this PDO and a signal index. This index holds
 * information about which signal from the input of this block will be transmitted with this
 * PDO.
 *
 * When enabled this block will transmit a sync package each time it runs.
 *
 * @tparam N - number of CAN nodes (2 - default)
 * @tparam M - each drive can have several floating point input signals (1 - default)
 *             e.g. velocity and position
 * @tparam P - each drive can have several integer input signals (1 - default)
 *             e.g. control word and digital output
 *
 * @since v1.2
 */
template < uint8_t N = 2, uint8_t M = 1, uint8_t P = 1 >
class CANopenSend : public Blockio<N,0,Matrix<M,1,double>> {
  
 public:
  /**
   * Constructs a CANopen send block instance for a given set of nodes.
   * Sets the scale of floating point inputs to 1.
   *
   * @param co - CANopen object
   * @param node - vector with node id's of all connected CAN nodes 
   */
  template<typename CanOpen, typename NodeList = std::initializer_list<uint8_t>>
  CANopenSend(CanOpen&& co, NodeList node)
      : co(co), node(node), log(Logger::getLogger('Y')) {
    for (size_t i = 0; i < node.size(); i++) {
      scale[i] = 1;
    }
    log.info() << "CAN send block constructed with " << node.size() << " nodes";
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  CANopenSend(const CANopenSend& s) = delete;

  /**
   * Transmits on the CAN bus. If enabled a sync package is sent for each run. After this the block
   * will send all configured RPDO's to each of the connected CAN nodes.
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() override {
    if (enabled) {
      int err;
      if ((err = co.sendSync()) != 0){
        std::ostringstream os; log.error() << "send sync failed: " << std::strerror(errno);
        throw eeros::Fault(os.str());
      };

      for (const auto& p : pdo) {
        uint8_t nodeNr = std::get<0>(p);
        uint8_t nodeId = std::get<1>(p);
        uint8_t RPDOnr = std::get<2>(p);
        uint8_t buf[8];
        auto length = std::get<3>(p);
        auto sigIdx = std::get<4>(p);
        int i = 0, len = 0;
        for (uint8_t l : length) {
          l /= 8; // lenght is in bits
          if (sigIdx[i] > 0) {
            Matrix<N,M,double> val = this->getIn(nodeNr).getSignal().getValue() * scale[nodeNr];
            co.can.encodeU32(buf+len, (uint32_t)val[sigIdx[i]-1], l);
          } else if (sigIdx[i] < 0) {
            Matrix<P,1,uint32_t> val = this->getDigIn(nodeNr).getSignal().getValue();
            co.can.encodeU32(buf+len, val[-sigIdx[i]-1], l);
          } else {
            co.can.encodeU32(buf+len, ctrl[nodeNr], l);
          }
          len += l;
          i++;
        }
        err = co.PDOsend(nodeId, RPDOnr, buf, len);
        if (err != 0) throw eeros::Fault("sending over CAN failed");
      }
    }
  }
        
  /**
   * Enables the block.
   *
   * If enabled, run() will send a sync package followed by PDOs onto the CAN bus.
   *
   * @see run()
   */
  virtual void enable() override {
    enabled = true;
  }
  
  /**
   * Disables the block.
   *
   * If disabled, no sync and PDOs will be sent.
   *
   * @see run()
   */
  virtual void disable() override {
    enabled = false;
  }

  /**
   * Sets the scaling.
   *
   * The drive needs its position and velocity information as 4 bytes counter values.
   * The scaling allows to transform this counter values from meaningful position and
   * velocity information in rad, m, rad/s or m/s.
   *
   * @param scale the scaling factor for the velocity for all drives
   */
  virtual void setScale(Matrix<N,M,double>& scale) {
    this->scale = scale;
  }

  /**
   * Sets the control word to be sent to a given drive.
   *
   * @param index index of the drive
   * @param ctrl control word
   */
  virtual void setCtrl(uint8_t index, uint16_t ctrl) {
    this->ctrl[index] = ctrl;
  }

  /**
   * Configures the PDO which are sent by this block.
   *
   * First of all this includes the node id of the slave drive. This id is searched for in the
   * list with all node ids and the corresponding index is saved as well for later use.
   * Next, you have to specify which RPDO number is chosen together with one or several
   * CANopen dictionary objects which are mapped into this RPDO. Last, you must chose for
   * each mapping, where the corresponding signal comes from.
   *
   * @param nodeId the id of the node to which this PDO is sent
   * @param RPDOnr nr of the RPDO (1 to 4)
   * @param objs opjects which are mapped into this PDO
   * @param idx signal index, where
   *            - > 0 : index in floating point inputs
   *            - 0   : control word
   *            - < 0 : index in integer inputs
   */
  virtual void configureRPDO(uint8_t nodeId, uint8_t RPDOnr, std::vector<coObject_t> objs, std::vector<int8_t> idx) {
    std::vector<uint8_t>::iterator it = std::find(node.begin(), node.end(), nodeId);
    if (it == node.end()) {
      log.warn() << "CAN configure RPDO: node id " << nodeId << " not found";
      return;
    }
    uint8_t nodeIdx = std::distance(node.begin(), it);
    std::vector<uint8_t> len;
    for (coObject_t o : objs) {
       len.push_back(o.len);
    }
    pdo.push_back(make_tuple(nodeIdx, nodeId, RPDOnr, len, idx));
  }

  /**
   * Get the input of the digital signals of the block.
   *
   * @param index - index of the input
   * @return input
   */
  virtual Input<Matrix<P,1,uint32_t>>& getDigIn(uint8_t index) {
    if (index >= N) throw IndexOutOfBoundsFault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
    return digIn[index];
  }

 private:
  CANopen co;
  bool enabled = false;
  Input<Matrix<P,1,uint32_t>> digIn[N];
  Matrix<N,1,uint16_t> ctrl;
  Matrix<N,M,double> scale;
  std::vector<uint8_t> node;
  // node index, node id, RPDOnr, length in bit of all objects, signal index of all objects
  std::vector<std::tuple<uint8_t,uint8_t,uint8_t,std::vector<uint8_t>,std::vector<int8_t>>> pdo;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANOPENSEND_
