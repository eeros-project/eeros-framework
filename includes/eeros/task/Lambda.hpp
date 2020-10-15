#ifndef ORG_EEROS_TASK_LAMBDA_HPP_
#define ORG_EEROS_TASK_LAMBDA_HPP_

#include <functional>
#include <eeros/core/Runnable.hpp>

namespace eeros {
namespace task {

/**
 * Helper class for runnables. In instance of this class allows to run a given function as a runnable.
 * 
 * @since v0.6
 */
class Lambda : public Runnable {
 public:
  /**
   * Constructs a default lambda instance. This runnable will run a function doing nothing. 
   */
  Lambda() : f([](){}) { }

  /**
   * Constructs a lambda instance which will run the given function.
   *
   * @param f - function to be run by the lambda
   */
  Lambda(std::function<void()> f) : f(f) { }

  /**
   * This method will be executed whenever this runnable runs.
   */
  virtual void run() { 
    f(); 
  }
  
 private:
  std::function<void()> f;
};

}
}

#endif // ORG_EEROS_TASK_LAMBDA_HPP_
