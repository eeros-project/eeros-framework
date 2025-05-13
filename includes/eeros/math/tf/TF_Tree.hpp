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
  virtual bool initJSON(std::string path);

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
  virtual bool initJSON(ucl::Ucl jsonParameter);

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
  void addTF(std::string name, std::string base, Matrix<4, 4, double> m = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                               0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0}) {
    std::vector<int> parents;
    int id = nodes.size();
    if (name != "global") {
      for (uint32_t i = 0; i < nodes.size(); i++) {
        if (nodes[i].getName() == name) throw TF_SameNameException(name);
      }
      TF_Node& node = getNode(base);
      parents = node.getParents();
      parents.push_back(node.getId());
      node.addChild(id);
    }
    TF_Node node(name, base, id, parents, m);
    nodes.push_back(node);
  }

  /**
   * Adds a node with a transformation matrix given by
   * a translation and rotation to a tf tree.
   *
   * @param name - name of the matrix
   * @param base - name of the parent node
   * @param trans - translation
   * @param rpy - rotation
   */
  void addTF(std::string name, std::string base,
                     Matrix<3, 1, double> trans,
                     Matrix<3, 1, double> rpy = {0.0, 0.0, 0.0}) {
    TF_Matrix tf;
    tf.setTrans(trans);
    tf.setRPY(rpy);
    addTF(name, base, tf.getMatrix());
  }

  /**
   * Returns the transformation matrix of the node
   * with a given name.
   *
   * @param name - name of the node
   *
   * @return transformation matrix
   */
  TF_Matrix& getTF(std::string name) {
    TF_Node& node = getNode(name);
    return node.getTF();
  }

  /**
   * Returns the transformation matrix of the node
   * with a given name.
   *
   * @param name - name of the node
   *
   * @return transformation matrix
   */
  virtual TF_Matrix trafoFromWithBase(std::string destName,
                                      std::string baseName) {
//     TF_Matrix trafo /*destName, baseName*/;
//         trafo.eye();
    //
    //     //// find needed TF in tf_list::
    //     int from = -1, to = -1;
    //     for (int i = 0; i < nodes.size(); i++) {
    //       if (nodes[i].getTF()->getName() == baseName) from = i;
    //       if (nodes[i].getTF()->getName() == destName) to = i;
    //       if (from >= 0 && to >= 0) break;
    //     }
    //     if (from == -1 || to == -1) {
    //       Logger::getLogger().warn()
    //           << "Error: TF_TREE::trafoFromToBase:  did not found needed TFs
    //           ";
    //     };
    //
    //     //// from list
    //     std::vector<int> fromList = nodes[from].getParents();
    //     fromList.push_back(from);
    //
    //     //// to list
    //     std::vector<int> toList = nodes[to].getParents();
    //     toList.push_back(to);
    //
    //     //// search nearest parent tf
    //     int sameParent, counter = 0;
    //     bool hasSameParent = true;
    //
    //     while (hasSameParent && counter < fromList.size() &&
    //            counter < toList.size()) {
    //       if (fromList[counter] == toList[counter]) {
    //         sameParent = fromList[counter];
    //         counter++;
    //       } else {
    //         hasSameParent = false;
    //       }
    //     }
    //
    //     //// count backward from "to" to "sameParent"
    //     counter = toList.size() - 1;
    //
    //     while (toList[counter] != sameParent) {
    //       trafo.setMatrix(nodes[toList[counter]].getTF()->getMatrix() *
    //                       trafo.getMatrix());
    //       counter--;
    //     }
    //
    //     //// count forward from "sameParent" to "from"
    //     while (fromList[counter] != from) {
    //       counter++;
    //       trafo.setMatrix(nodes[fromList[counter]].getTF()->inv() *
    //                       trafo.getMatrix());
    //     }

//     return trafo;
  }

  /**
   * Print tree from given node
   */
  virtual void print(std::string start = "global", bool showTF = false) {
    static int printLevel = 0;
    if (printLevel == 0) {
      Logger::getLogger().info() << "Show TF_Tree from node : " << start;
    }
    TF_Node& node = getNode(start);
    int id = node.getId();
    for (uint32_t i = 0; i < node.getChildren().size(); i++) {
      std::string str;
      for (uint32_t k = 0; k < node.getParents().size(); k++) {
        str += "  ";
      }
      TF_Node child = nodes[node.getChildren()[i]];
      str += "  -> " + child.getName();
      auto tf = child.getTF();
      if (showTF) {
        str += ":   Trans: [ " + std::to_string(tf.getTrans()(0)) + ", " +
               std::to_string(tf.getTrans()(1)) + ", " +
               std::to_string(tf.getTrans()(2)) + " ] , RPY [ " +
               std::to_string(tf.getRPY()(0)) + ", " +
               std::to_string(tf.getRPY()(1)) + ", " +
               std::to_string(tf.getRPY()(2)) + " ] ";
      }
      Logger::getLogger().info() << str;

      if (nodes[nodes[id].getChildren()[i]].getChildren().size() > 0) {
        printLevel++;
        print(nodes[nodes[id].getChildren()[i]].getName(), showTF);
        printLevel--;
      }
    }
  }

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
