#pragma once

#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/math/tf/TF_Matrix.hpp>
#include <string>
#include <vector>

namespace eeros {
namespace math {
namespace tf {

using namespace eeros::math;

/**
 * Each node holds a transformation matrix together with information about
 * its parent nodes and child nodes. The list of the parents include all
 * the nodes in the tree up to the root, which is the node with the name
 * "global".
 */
class TF_Node {
 public:
  /**
   * Creates a new node which is either the root ("global")
   * or a leaf.
   *
   * @param name - name of the matrix
   * @param base - name of the base
   * @param id - id of this node
   * @param parents - list of all parents of this node
   * @param m - init values
   */
  TF_Node(std::string name, std::string base, int id, std::vector<int> parents,
          Matrix<4, 4, double> m = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0})
      : name(name), baseName(base), id(id), parents(parents), tf(m) {
    if (name == "global") parents.clear();
    else this->parents = parents;
  }

  virtual ~TF_Node() {}

  /**
   * Gets the name of the transformation.
   *
   * @return name
   */
  virtual std::string getName() const { return name; };

  /**
   * Gets the name of the base of the transformation.
   *
   * @return base name
   */
  virtual std::string getBaseName() const { return baseName; };

  /**
   * Returns the id of a node.
   *
   * @return id of this node
   */
  const int getId() const { return id; }

  /**
   * Returns the list of the parents of this node.
   *
   * @return list of parents
   */
  const std::vector<int> getParents() const { return parents; }

  /**
   * Returns the transformation matrix of this node.
   *
   * @return transformation matrix
   */
  TF_Matrix& getTF() { return tf; }

  /**
   * Add a child to a node.
   *
   * @param id - id of the child
   */
  void addChild(const int id) { children.push_back(id); }

  /**
   * Returns the list with the children of this node.
   *
   * @return list of children
   */
  std::vector<int> getChildren() const { return children; }

 private:
  std::string name, baseName;
  int id;
  std::vector<int> parents;  // holds parent ids up to the root
  std::vector<int> children;
  TF_Matrix tf;
};

}  // namespace tf
}  // namespace math
}  // namespace eeros
