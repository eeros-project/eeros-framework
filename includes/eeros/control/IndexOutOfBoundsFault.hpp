#ifndef ORG_EEROS_CONTROL_INDEXOUTOFBOUNDSFAULT_HPP
#define ORG_EEROS_CONTROL_INDEXOUTOFBOUNDSFAULT_HPP

#include <eeros/core/Fault.hpp>

namespace eeros {
namespace control {
  
class IndexOutOfBoundsFault : public eeros::Fault {
 public:
  IndexOutOfBoundsFault();
  explicit IndexOutOfBoundsFault(std::string m);
  virtual ~IndexOutOfBoundsFault() throw();
};

};
};

#endif // ORG_EEROS_CONTROL_INDEXOUTOFBOUNDSFAULT_HPP
