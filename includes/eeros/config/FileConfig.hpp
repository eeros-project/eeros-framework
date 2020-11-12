#ifndef ORG_EEROS_CONFIG_FILECONFIG_HPP_
#define ORG_EEROS_CONFIG_FILECONFIG_HPP_

#include <eeros/core/Fault.hpp>
#include <eeros/config/Config.hpp>
#include <fstream>

namespace eeros {
namespace config {

/** \brief Configuration saved to and loaded from file.
 *
 * This allows for saving a configuration to a file on disk or loading it from there.
 * A configuration might be useful to keep calibration values or setup data.
 */
class FileConfig : public Config {
 public:

  /**
   * Creates a configuration
   * @param path Name of the configuration file.
   */
  FileConfig(std::string path = nullptr) : Config(path) { }
  
  /**
   * Destructor, do not call manually.
   */
  virtual ~FileConfig() { }
      
  /**
   * Saves a configuration to a file.
   * @param path Name of the configuration file
   * @return true if successful
   */
  virtual bool save(std::string path = "") {
    if (path.empty()) path = this->path;
    if (path.empty()) throw Fault("path is null");
    std::ofstream file(path);
    if (file.fail()) return false;
    for (auto p: properties ) {
      std::ostringstream os;
      os << p.first << " = ";
      std::string val;
      p.second.set(p.first, val);
      os << val << std::endl;
      file << os.str();
    }
    return true;
  }
 
  /**
   * Loads a configuration from a file.
   * @param path Name of the configuration file
   * @return true if successful
   */
  virtual bool load(std::string path = "") {
    if (path.empty()) path = this->path;
    if (path.empty()) throw Fault("path is null");
    std::ifstream file(path);
    if (file.fail()) return false;
    std::string line;
    while (getline(file, line)) {
      std::stringstream iss(line);
      std::string key, val;
      iss >> key >> val;  
      getline(iss, val);
  
      auto p = properties.find(key);
      if (p == properties.end()) {
        continue; // unknown property
      }
      p->second.get(p->first, val);
    }
    return true;
  }
};
  
}
}

#endif // ORG_EEROS_CONFIG_FILECONFIG_HPP_
