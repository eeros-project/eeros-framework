#include <eeros/math/tf/TF_Tree.hpp>

using namespace eeros::math::tf;

bool TF_Tree::initJSON(std::string path) {
  std::string err;
  log.info() << "TF_Tree: read json file : " << path;
  ucl::Ucl jsonParameter =
      ucl::Ucl::parse_from_file_strategy(path, UCL_DUPLICATE_ERROR, err);
  return initJSON(jsonParameter);
}

bool TF_Tree::initJSON(const char* path) { return initJSON(std::string(path)); }

bool TF_Tree::initJSON(ucl::Ucl jsonParameter) {
  std::string err;
  if (jsonParameter) {
    // init TrafoMatrix in "tfList"
    for (uint32_t i = 0; i < jsonParameter["tfList"].size(); i++) {
      // std::string name = jsonParameter["tfList"][i]["name"].string_value();
      // Logger::getLogger().info() << "new TF : " << name;
      Matrix<3, 1, double> trans = {
          jsonParameter["tfList"][i]["trans"][0].number_value(),
          jsonParameter["tfList"][i]["trans"][1].number_value(),
          jsonParameter["tfList"][i]["trans"][2].number_value()};
      Matrix<3, 1, double> rotRPY = {
          jsonParameter["tfList"][i]["rotRPY"][0].number_value(),
          jsonParameter["tfList"][i]["rotRPY"][1].number_value(),
          jsonParameter["tfList"][i]["rotRPY"][2].number_value()};
      try {
        addTF(jsonParameter["tfList"][i]["name"].string_value(),
              jsonParameter["tfList"][i]["parent"].string_value(), trans,
              rotRPY);
      } catch (TF_SameNameException& e) {
        log.warn() << e.what();
      } catch (TF_NotFoundException& e) {
        log.warn() << "In initJSON: tfList : Could not create TF "
                   << jsonParameter["tfList"][i]["name"].string_value()
                   << " because parent " << e.getName() << " does not exist";
      }
    }
  }
  return true;
}

void TF_Tree::addTF(const std::string name, const std::string base,
                    const Matrix<4, 4, double> m) {
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

void TF_Tree::addTF(const std::string name, const std::string base,
                    const Matrix<3, 1, double> trans,
                    const Matrix<3, 1, double> rpy) {
  TF_Matrix tf;
  tf.setTrans(trans);
  tf.setRPY(rpy);
  addTF(name, base, tf.getMatrix());
}

TF_Matrix& TF_Tree::getTF(const std::string name) {
  TF_Node& node = getNode(name);
  return node.getTF();
}

TF_Matrix TF_Tree::tfFromTo(const std::string start, const std::string dest) {
  int from = -1, to = -1;
  for (int i = 0; i < (int)nodes.size(); i++) {
    if (nodes[i].getName() == start) from = i;
    if (nodes[i].getName() == dest) to = i;
    if (from >= 0 && to >= 0) break;
  }
  if (from == -1) throw(TF_NotFoundException(start));
  if (to == -1) throw(TF_NotFoundException(dest));
  std::vector<int> fromList = nodes[from].getParents();
  fromList.push_back(from);
  std::vector<int> toList = nodes[to].getParents();
  toList.push_back(to);
  // search nearest parent
  int sameParent, cnt = 0;
  bool hasSameParent = true;
  while (hasSameParent && cnt < (int)fromList.size() &&
         cnt < (int)toList.size()) {
    if (fromList[cnt] == toList[cnt]) {
      sameParent = fromList[cnt];
      cnt++;
    } else {
      hasSameParent = false;
    }
  }
  TF_Matrix tf;
  tf.eye();
  // count backward from "to" to "sameParent"
  cnt = toList.size() - 1;
  while (toList[cnt] != sameParent) {
    tf.setMatrix(nodes[toList[cnt]].getTF() * tf.getMatrix());
    cnt--;
  }
  // count forward from "sameParent" to "from"
  while (fromList[cnt] != from) {
    cnt++;
    tf.setMatrix(nodes[fromList[cnt]].getTF().inverse() * tf.getMatrix());
  }
  return tf;
}

void TF_Tree::print(std::string start, bool showTF) {
  static int printLevel = 0;
  if (printLevel == 0) {
    log.info() << "Show TF_Tree from node : " << start;
  }
  TF_Node& node = getNode(start);
  int id = node.getId();
  for (uint32_t i = 0; i < node.getChildren().size(); i++) {
    std::stringstream ss;
    std::string str;
    for (uint32_t k = 0; k < node.getParents().size(); k++) {
      ss << "  ";
    }
    TF_Node child = nodes[node.getChildren()[i]];
    ss << "  -> " << child.getName();
    auto tf = child.getTF();
    if (showTF) {
      ss << ":   " << tf;
    }
    log.info() << ss.str();

    if (nodes[nodes[id].getChildren()[i]].getChildren().size() > 0) {
      printLevel++;
      print(nodes[nodes[id].getChildren()[i]].getName(), showTF);
      printLevel--;
    }
  }
}
