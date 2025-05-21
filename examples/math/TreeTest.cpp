#include <eeros/math/tf/TF_Tree.hpp>
#include <eeros/logger/Logger.hpp>

using namespace eeros::math::tf;
using namespace eeros::logger;

int main(void){
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();

  log.info() << "start TF_Tree_Test: ";

  TF_Tree& tfTree = TF_Tree::instance();
  tfTree.initJSON("Tree.json");
  tfTree.getTF("center").calcRPYfromRot();
  tfTree.getTF("head").calcRPYfromRot();
  tfTree.getTF("eyeLeft").calcRPYfromRot();
  tfTree.getTF("eyeRight").calcRPYfromRot();
  tfTree.getTF("shoulderLeft").calcRPYfromRot();
  tfTree.getTF("shoulderRight").calcRPYfromRot();
  tfTree.getTF("ellbowLeft").calcRPYfromRot();
  tfTree.getTF("ellbowRight").calcRPYfromRot();
  tfTree.getTF("handLeft").calcRPYfromRot();
  tfTree.getTF("handRight").calcRPYfromRot();
  tfTree.print("global", true);

  return 0;
}

