#pragma once

#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
// #include <eeros/math/tf/TF_Matrix.hpp>
#include <ucl++.h>

#include <eeros/math/tf/TF_Exceptions.hpp>
#include <eeros/math/tf/TF_Node.hpp>
#include <exception>
#include <string>
#include <vector>

namespace eeros {
namespace math {
namespace tf {

using namespace eeros::math;
using namespace eeros::logger;

/**
 * A tree consisting of nodes containing transformation matrices
 */
class TF_Tree {
 private:
  TF_Tree() : log(Logger::getLogger()) {
    addTF("global", "");
  }

 public:
  /**
   * Gets an instance of a tf tree. Will be created if none exists.
   * A newly created tree consists of a root node holding the identity matrix.
   *
   * @return tf_tree
   */
  static TF_Tree& instance() {
    static TF_Tree tfTree;
    return tfTree;
  }

  /**
   * Reads tf tree from configuration file.
   *
   * @param path - path containing the file
   *
   * @return true, if successful
   */
  virtual bool initJSON(const std::string path);

  /**
   * Reads tf tree from configuration file.
   *
   * @param path - path containing the file
   *
   * @return true, if successful
   */
  virtual bool initJSON(const char* path);

  /**
   * Reads tf tree from configuration file.
   *
   * @param jsonParameter - json parameter
   *
   * @return true, if successful
   */
  virtual bool initJSON(const ucl::Ucl jsonParameter);

  /**
   * Adds a node containing a transformation matrix to a tf tree.
   * The node gets an id, which is equal to the number of added
   * nodes. A node can only be created as root or leaf.
   * It cannot be inserted into an existing tree except as a leaf.
   *
   *
   * @param name - name of the node
   * @param base - name of the parent node
   * @param m - transformation matrix
   */
  void addTF(const std::string name, const std::string base, const Matrix<4, 4, double> m = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0});
  /**
   * Adds a node with a transformation matrix given by
   * a translation and rotation to a tf tree.
   *
   * @param name - name of the matrix
   * @param base - name of the parent node
   * @param trans - translation
   * @param rpy - rotation
   */
  void addTF(const std::string name, const std::string base,
                     const Matrix<3, 1, double> trans,
                     const Matrix<3, 1, double> rpy = {0.0, 0.0, 0.0});

  /**
   * Returns the transformation matrix of the node
   * with a given name.
   *
   * @param name - name of the node
   *
   * @return transformation matrix
   */
  TF_Matrix& getTF(const std::string name);

  /**
   * Returns the transformation matrix leading from a start node
   * to a destination node. Both nodes must be present in this tree.
   * The overall transormation is calculated by searching for a common
   * parent node.
   *
   * @param start - name of the start node
   * @param dest - name of the start node
   *
   * @return transformation matrix
   */
  virtual TF_Matrix tfFrameToOrigin(const std::string frame,
                                const std::string origin);

  /**
   * Print tree from given node
   *
   * @param start - starting node
   * @param showTF - true -> print translation and rpy
   */
  virtual void print(std::string start = "global", bool showTF = false);

 protected:
  virtual TF_Node& getNode(std::string name) {
    for (uint32_t i = 0; i < nodes.size(); i++) {
      if (nodes[i].getName() == name) {
        return nodes[i];
      }
    }
    throw(TF_NotFoundException(name));
  }

 private:
  ucl::Ucl jsonParameter;
  std::vector<TF_Node> nodes;
  Logger log;
};

}  // namespace tf
}  // namespace math
}  // namespace eeros
