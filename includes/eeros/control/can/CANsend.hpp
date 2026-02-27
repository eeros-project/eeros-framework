#ifndef ORG_EEROS_CONTROL_CANSEND_
#define ORG_EEROS_CONTROL_CANSEND_

#include <eeros/logger/Logger.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <CAN.hpp>

using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::math;

namespace eeros {
namespace control {

/**
 * This block serves to send CAN messages to a MYACTUATOR RMD-X series drive.
 *
 * @tparam N - number of CAN nodes (2 - default)
 *
 * @since v3.2
 */
template < uint8_t N = 2 >
class CANsend : public Blockio<1,0,Matrix<N,1,double>> {
 public:
  /**
   * Constructs a CAN send block instance for a given set of nodes.
   * Sets the scale of position to 1.
   *
   * @param handle - handle to socket CAN connection
   * @param node - vector with node id's of all connected CAN nodes
   */
  CANsend(CAN& handle, std::initializer_list<uint8_t> node)
      : canHandle(handle), nodes(node), log(Logger::getLogger('Y')) {
    for (size_t i = 0; i < node.size(); i++) {
      scale[i] = 1;
    }
    log.info() << "CAN send block constructed, " << node.size() << " nodes";
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  CANsend(const CANsend& s) = delete;

  /**
   * Transmits on the CAN bus. If enabled a CAN frame with data load of 8 bytes is sent
   * to each CAN node.
   *
   * @see enable()
   * @see disable()
   */
  virtual void run() override {
    if (enabled) {
      for (std::size_t i = 0; i < nodes.size(); i++) {
        uint32_t pos = this->getIn().getSignal().getValue()[i] * scale(i);
        uint16_t sp = speed[i];
        can_frame f;
        f.can_id = 0x140 + nodes[i];
        f.len = 8;
        f.data[0] = 0xa5;
        canHandle.encodeU32(&f.data[2], sp, 2);
        canHandle.encodeU32(&f.data[4], pos, 4);
        int err = canHandle.frameSend(f);
        if (err != 0) throw eeros::Fault("sending over CAN failed");
      }
    }
  }

  /**
   * Enables the block.
   *
   * If enabled, run() will send onto the CAN bus.
   *
   * @see run()
   */
  virtual void enable() override {
    enabled = true;
  }

  /**
   * Disables the block.
   *
   * If disabled, nothing will be sent.
   *
   * @see run()
   */
  virtual void disable() override {
    enabled = false;
  }

  /**
   * Sets the maximum speed.
   *
   * The speed is a 2 bytes value.
   *
   * @param speed maximum speed
   */
  virtual void setSpeed(Matrix<N,1,double>& speed) {
    this->speed = speed;
  }

  /**
   * Sets the scaling for the position information.
   *
   * The drive needs its position information as a 4 bytes value.
   * The scaling allows to transform this value into meaningful
   * position information in rad/s or m/s.
   *
   * @param scale The scaling factor for the position for all drives
   */
  virtual void setScale(Matrix<N,1,double>& scale) {
    this->scale = scale;
  }

 private:
  CAN& canHandle;
  bool enabled = false;
  Matrix<N,1,double> scale;
  Matrix<N,1,double> speed;
  std::vector<uint8_t> nodes;
  Logger log;
};

}
}

#endif // ORG_EEROS_CONTROL_CANSEND_



