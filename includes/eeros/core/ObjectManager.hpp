#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <any>
#include <eeros/logger/Logger.hpp>

/**
 * This Class is to centralize the object management
 * 
 * When using this class, make sure that all objects are not raw pointers.
 * Use shared pointers for all objects managed by this class.
 * 
 * This class holds all objects (as shared pointer<std::any>)
 * Each stored object is associated with a key.
 * This key must be used to retrieve the object.
*/
class ObjectManager {
public:
  // Singleton
  static ObjectManager* getInstance() {
    static ObjectManager* instance = new ObjectManager();
    return instance;
  }

  ObjectManager(ObjectManager&&) = delete;
  ObjectManager(const ObjectManager&) = delete;
  ObjectManager& operator=(ObjectManager&&) = delete;
  ObjectManager& operator=(const ObjectManager&) = delete;

  /**
   * @brief Gets an object from the Manager wich is store by the key
   * 
   * @warning This function is expensive. Do not call this function outside from the init.
   * Especialy not in run methodes or similar.
   * 
   * @param key The key for the requested object.
   * @return the object witch belongs to the key.
   * 
   * @throw std::bad_any_cast is thrown if template is not the same as the stored type.
   * @throw std::runtime_error is thrown if key is not in the list.
  */
  template<typename T>
  std::shared_ptr<T> get(const std::string& key) {
    auto it = objects.find(key);
    if (it != objects.end()) {
      try {
        std::shared_ptr<T> storedPtr = std::any_cast<std::shared_ptr<T>>(*(it->second));
        return storedPtr;
      } catch (const std::bad_any_cast& e) {
        log.error() << "Failed to cast: " << e.what();
        /* Pass it to the global abort method, or let the user catch it. */
        throw e;
      }
    }
    std::ostringstream oss;
    oss << __FILE__ << "::" << __LINE__ << "\n\t\t\t\t\t ObjectManager: Key \"" << key << "\" not found inside the object list.";
    throw std::runtime_error(oss.str());
  }

  /**
   * Sets an object to the Manager which is store by the key
   * 
   * @param key The key to store the object
   * @param obj The object to store
  */
  template<typename T>
  void add(const std::string& key, std::shared_ptr<T> smartPtr) {
    if(objects.count(key)) {
      log.fatal() << "ObjectManager: Key: \"" << key << "\" already exists inside the list.";
    }
    objects[key] = std::make_shared<std::any>(smartPtr);
  }

  /**
   * @see addObject(const std::string&, std::shared_ptr<T>)
  */
  template<typename T>
  void add(const std::string& key, T* obj) {
    std::shared_ptr<T> smartPtr(obj);
    add(key, smartPtr);
  }

  /**
   * @brief Get the number of stored objects
   * @return nof Objects
  */
  size_t nofObjects(){return objects.size();}

  /**
   * @brief Givs a list of all keys back
   * @return List of all keys
  */
  std::vector<std::string> storedKeys() {
    std::vector<std::string> keys;
    keys.reserve(objects.size());
    for (const auto& pair : objects) {
        keys.push_back(pair.first);
    }
    return keys;
  }

private:
  ObjectManager() = default;

  std::unordered_map<std::string, std::shared_ptr<std::any>> objects;
  eeros::logger::Logger log = eeros::logger::Logger::getLogger('O');
};