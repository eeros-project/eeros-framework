#ifndef ORG_EEROS_CORE_RUNNABLE_HPP_
#define ORG_EEROS_CORE_RUNNABLE_HPP_

namespace eeros {

/**
 * This is the base class for all runnables such as blocks, time domains, safety system, harmonics, etc.
 * 
 * @since v0.4
 */
class Runnable {
 public:
  virtual ~Runnable();
  
  /**
   * This method will be executed whenever a runnable runs.
   */
  virtual void run() = 0;
};

};

#endif // ORG_EEROS_CORE_RUNNABLE_HPP_
