#include <eeros/config/Config.hpp>
#include <eeros/core/Fault.hpp>

#include <sstream>

using namespace eeros::config;

void Config::add(std::string name, int &value) {
  auto k = properties.find(name);
  if (k != properties.end()) {
    throw eeros::Fault(std::string("Property '") + name + "' already added.");
  }
  properties[name] = ConfigPropertyAccessor {
    [&value] (std::string name, std::string& val) -> void {
      val = std::to_string(value);
    },
    [&value] (std::string name, const std::string& val) -> void {
      value = std::stoi(val);
    }
  };
}

void Config::add(std::string name, double &value) {
  auto k = properties.find(name);
  if (k != properties.end()) {
    throw eeros::Fault(std::string("Property '") + name + "' already added.");
  }
  properties[name] = ConfigPropertyAccessor {
    [&value] (std::string name, std::string& val) -> void {
      val = std::to_string(value);
    },
    [&value] (std::string name, const std::string& val) -> void {
      value = std::stod(val);
    }
  };
}

void Config::add(std::string name, std::size_t length, int *start, int *end, int defaultValue) {
  if (start + length != end) {
    throw eeros::Fault(std::string("Property '") + name + "': array length inconsistent");
  }
  auto k = properties.find(name);
  if (k != properties.end()) {
    throw eeros::Fault(std::string("Property '") + name + "' already added.");
  }
  properties[name] = ConfigPropertyAccessor{
    [length, start] (std::string name, std::string& val) -> void {
      std::stringstream ss;
      for (std::size_t i = 0; i < length; i++) {
        if (i != 0) ss << ", ";
        ss << start[i];
      }
      val = ss.str();
    },
    [length, start, defaultValue] (std::string name, const std::string& val) -> void {
      std::size_t i = 0;
      std::stringstream ss(val);
      std::string el;
      while (getline(ss, el, ',')) {
        int v = std::stoi(el);
        start[i++] = v;
      }
      while (i < length) {
        start[i++] = defaultValue;
      }
    }
  };
}

void Config::add(std::string name, std::size_t length, double *start, double *end, double defaultValue) {
  if (start + length != end) {
    throw eeros::Fault(std::string("Property '") + name + "': array length inconsistent");
  }
  auto k = properties.find(name);
  if (k != properties.end()) {
    throw eeros::Fault(std::string("Property '") + name + "' already added.");
  }
  properties[name] = ConfigPropertyAccessor {
    [length, start] (std::string name, std::string& val) -> void {
      std::stringstream ss;
      for (std::size_t i = 0; i < length; i++) {
        if (i != 0) ss << ", ";
        ss << start[i];
      }
      val = ss.str();
    },
    [length, start, defaultValue] (std::string name, std::string val) -> void {
      std::size_t i = 0;
      std::stringstream ss(std::move(val));
      std::string el;
      while (getline(ss, el, ',')) {
        double v = std::stod(el);
        start[i++] = v;
      }
      while (i < length) {
        start[i++] = defaultValue;
      }
    }
  };
}

void Config::add(std::string name, std::string &value) {
  auto k = properties.find(name);
  if (k != properties.end()) {
    throw eeros::Fault(std::string("Property '") + name + "' already added.");
  }
  properties[name] = ConfigPropertyAccessor {
    [&value] (std::string name, std::string& val) -> void {
      val = value;
    },
    [&value] (std::string name, const std::string_view val) -> void {
      if (val.size() > 0) value = val.substr(1,val.size() - 1);
      else value = "";
    }
  };
}
