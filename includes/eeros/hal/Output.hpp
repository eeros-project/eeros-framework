#ifndef ORG_EEROS_HAL_OUTPUT_HPP_
#define ORG_EEROS_HAL_OUTPUT_HPP_

#include <eeros/core/System.hpp>
#include <string>
#include <eeros/SIUnit.hpp>

namespace eeros {
namespace hal {

/**
 * Base class for all output classes
 */
class OutputInterface {
 public:
  virtual ~OutputInterface() { }
  virtual std::string getId() const = 0;
  virtual void* getLibHandle() = 0;
};

template <typename T>
class Output : public OutputInterface {
 public:
  explicit Output(std::string id, void* libHandle) : id(id), libHandle(libHandle) { }
  virtual ~Output() { }
  virtual inline std::string getId() const { return id; }
  virtual SIUnit getUnit() { return SIUnit::create(); }
  virtual T get() = 0;
  virtual void set(T value) = 0;
  virtual void setTimestampSignalIn(uint64_t timestampNs) { return; }
  virtual void *getLibHandle() { return libHandle; }

  T safe;
 private:
  std::string id;
  void *libHandle;
};

}
}

#endif /* ORG_EEROS_HAL_OUTPUT_HPP_ */
