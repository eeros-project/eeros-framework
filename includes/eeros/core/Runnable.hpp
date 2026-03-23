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
  /**
   * Virtual default destructor, created to ensure that if a pointer to this class is used and deleted, we will also call the derived base class destructor.
   * Ensure to override on derived base classes!
   * 
   * @note Deleting a base class destructor that does not have a virtual destructor is undefined behaviour, because the derived class destructor originally instantiated with new is never called.
   * This can cause potential memory leaks, because derived classes can not clean up their internal members as expected and instead simply leak them.
   */
  virtual ~Runnable();

  /**
   * This method will be executed whenever a runnable runs.
   */
  virtual void run() = 0;
};

};

#endif // ORG_EEROS_CORE_RUNNABLE_HPP_
