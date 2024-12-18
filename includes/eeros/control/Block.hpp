#ifndef ORG_EEROS_CONTROL_BLOCK_HPP_
#define ORG_EEROS_CONTROL_BLOCK_HPP_

#include <string>
#include <eeros/core/Runnable.hpp>

namespace eeros {
namespace control {

/**
 * This is the base class for all blocks used in a control system.
 * 
 * @since v0.4
 */

class Block : public Runnable {
 public:
  /**
   * Sets the name of the block.
   * 
   * @tparam name - name of the block
   */
  virtual void setName(std::string name);

  /**
   * Gets the name of the block.
   * 
   * @return name
   */
  virtual std::string getName() const;

  virtual void enable() {}
  virtual void disable() {}
  
 private:
  std::string name;
};

};
};

#endif /* ORG_EEROS_CONTROL_BLOCK_HPP_ */
