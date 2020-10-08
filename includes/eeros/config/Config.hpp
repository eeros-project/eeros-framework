#ifndef ORG_EEROS_CORE_CONFIG_HPP_
#define ORG_EEROS_CORE_CONFIG_HPP_

#include <cmath>
#include <map>
#include <functional>

namespace eeros {
namespace config {

struct ConfigPropertyAccessor {
  std::function<void(const std::string, std::string&)> set;
  std::function<void(const std::string, const std::string)> get;
};

struct StringCompare {
  bool operator()(const std::string first, const std::string second) {
    return first.compare(second) < 0;
  }
};

/** \brief Configuration.
 *
 * This is the base class for a configuration to be saved to or loaded from disk.
 * A configuration might be useful to keep calibration values or setup data.
 */
class Config {
 public:
  /**
   * Creates a configuration
   * @param path Name of the configuration.
   */
  Config(std::string path) : path(path) { };

  /**
   * Destructor, do not call manually.
   */
  virtual ~Config() { };
    
  /**
   * Loads a default configuration, can be overwritten by a derived class.
   */
  virtual void loadDefaults() { }

  /**
   * Saves a configuration, must be overwritten by a derived class
   * @param path Name of the configuration.
   * @return true if successful
   */
  virtual bool save(std::string path = "") = 0;

  /**
   * Loads a configuration, must be overwritten by a derived class
   * @param path Name of the configuration.
   * @return true if successful
   */
  virtual bool load(std::string path = "") = 0;
    
 protected:
  virtual void add(const std::string name, int &value);
  virtual void add(const std::string name, double &value);
  virtual void add(const std::string name, std::size_t length, int *start, int *end, int defaultValue = -1);
  virtual void add(const std::string name, std::size_t length, double *start, double *end, double defaultValue = NAN);
  virtual void add(const std::string name, std::string &value);

  template < typename T, std::size_t N >
  void add(const std::string name, std::array<T,N> &value);
    
  std::string path;
  std::map<std::string, ConfigPropertyAccessor, StringCompare> properties;
};


template < typename T, std::size_t N >
void Config::add(const std::string name, std::array<T,N> &value) {
  add(name, N, value.begin(), value.end());
}

}
}

#endif // ORG_EEROS_CORE_CONFIG_HPP_
