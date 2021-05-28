#ifndef ORG_EEROS_CORE_FAULT_HPP
#define ORG_EEROS_CORE_FAULT_HPP

#include <string>
#include <sstream>
#include <exception>

namespace eeros {

/**
 * Base class for EEROS faults. 
 */

class Fault : public std::exception {
 public:
  /**
   * Default constructor for a fault.
   */
  Fault();
  
  /**
   * Default constructor for a named fault.
   * 
   * @param m - name of the fault
   */
  explicit Fault(std::string m);
  
  /**
   * Default constructor for a fault.
   */
  virtual ~Fault() throw();
  
  /** Returns a C-style character string describing the general cause
   *  of the current error.  
   */
  virtual const char* what() const throw();

 protected:
  std::string message;
};

}

#endif // ORG_EEROS_CORE_FAULT_HPP
