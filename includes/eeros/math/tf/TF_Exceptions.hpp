#pragma once

#include <eeros/core/Fault.hpp>

namespace eeros {
namespace math {
namespace tf {

class TF_NotFoundException : public eeros::Fault {
 public:
  TF_NotFoundException() {}
  TF_NotFoundException(std::string name) : name(name) {
    message = "TF_Tree: expected node \"" + name + "\"  could not be found";
  }
  ~TF_NotFoundException() throw() {}
  virtual std::string getName() { return name; }

 private:
  std::string name;
};

class TF_SameNameException : public eeros::Fault {
 public:
  TF_SameNameException() {}
  explicit TF_SameNameException(std::string name) {
    message = "TF_Tree: could not create node because a node with name \"" + name +
              "\" already exists ";
  }
  ~TF_SameNameException() throw() {}
};

}  // namespace tf
}  // namespace math
}  // namespace eeros
