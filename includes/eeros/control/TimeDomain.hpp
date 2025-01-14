#ifndef ORG_EEROS_CONTROLTIMEDOMAIN_HPP
#define ORG_EEROS_CONTROLTIMEDOMAIN_HPP

#include <list>
#include <string>
#include <eeros/control/Block.hpp>
#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/NaNOutputFault.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyLevel.hpp>

namespace eeros {
namespace control {

using namespace safety;

/**
 * A timedomain is responsible for running the blocks which were added to it.
 * Blocks can be freely added and removed. A removed block will no longer be
 * run. A timedomain must be assigned a period. As soon as the timedomain is 
 * added to the executor, the executor will run the timedomain with the assigned 
 * period. You can stop the executor running the timedomain and restart it later.
 * 
 * @since v0.4
 */

class TimeDomain : public virtual Runnable {
 public:
  /**
   * Constructs a timedomain.
   *
   * @param name - name 
   * @param period - periodicity of the execution 
   * @param realtime - when true, executor creates a realtime thread if available by the system 
   */
  TimeDomain(std::string name, double period, bool realtime);
  
  /**
   * Adds a block to a time domain.
   *
   * @param block - block 
   */
  virtual void addBlock(Block* block);

  /**
   * Adds a block to a time domain.
   *
   * @param block - block 
   */
  virtual void addBlock(Block& block);

  /**
   * Removes a block from a time domain.
   *
   * @param block - block 
   */
  virtual void removeBlock(Block* block);

  /**
   * Removes a block from a time domain.
   *
   * @param block - block 
   */
  virtual void removeBlock(Block& block);
  
  /**
   * Returns the name of the timedomain.
   *
   * @return name 
   */
  std::string getName();

  /**
   * Returns the period of the timedomain.
   *
   * @return period 
   */
  double getPeriod();

  /**
   * Returns the true if the timedomain is executed with a realtime thread.
   *
   * @return true, if realtime 
   */
  bool getRealtime();

  /**
   * A timedomain can run into problems if blocks or connections between them are misconfigured.
   * In such cases the timedomain can fire safety events to notify the safety system.
   *
   * @param ss - reference to the safety system 
   * @param e - safety event 
   */
  void registerSafetyEvent(SafetySystem& ss, SafetyEvent& e);

  /**
   * The basic algorithm of the timedomain. It will run all blocks.
   */
  virtual void run();

  /**
   * Start the timedomain. Upon adding the timedomain to the executor, it will start
   * automatically. Call this method to restart the timedomain after having it stopped.
   *
   * @see stop()
   */
  virtual void start();

  /**
   * Stop the timedomain. 
   * 
   * @see start()
   */
  virtual void stop();

  /**
   * Enable all blocks in the domain
   *
   * @see Block::enable()
   */
  void enableBlocks();
  /**
   * Disable all blocks in the domain
   *
   * @see Block::disable()
   */
  void disableBlocks();

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  friend std::ostream& operator<<(std::ostream& os, TimeDomain& td);
  
 private:
  std::string name;
  double period;
  bool realtime;
  bool running = true;
  std::list<Block*> blocks;
  SafetySystem* safetySystem;
  SafetyEvent* safetyEvent;
};

}
}

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
