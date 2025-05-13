#include <eeros/math/tf/TF_Tree.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::math::tf;

// Testing constructors with initialization values
TEST(TF_TreeTest, init1) {
  TF_Tree& tfTree = TF_Tree::instance();
  tfTree.addTF("center", "global");
  tfTree.addTF("head", "center");
  tfTree.addTF("shoulderLeft", "center");
  tfTree.addTF("shoulderRight", "center");
  tfTree.addTF("eyeLeft","head");
  tfTree.addTF("eyeRight","head");
  tfTree.addTF("ellbowLeft", "shoulderLeft");
  tfTree.addTF("handLeft","ellbowLeft");
  tfTree.addTF("ellbowRight", "shoulderRight");
  tfTree.addTF("handRight","ellbowRight");

  try {
    tfTree.addTF("center", "head");
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("TF_Tree: could not create TF because a TF with name \"center\" already exists "));
  }

  tfTree.getTF("head").setTrans(Vector3(0.0, 0.0, 0.7));
  tfTree.getTF("head").setRPY(0.1, -0.1, 0.12);
  tfTree.getTF("eyeLeft").setTrans(Vector3(0.06, 0.02, 0.1));
  tfTree.getTF("eyeLeft").setRPY(0.0, 0.0, 0.2);
  tfTree.getTF("eyeRight").setTrans(eeros::math::Vector3(0.06, -0.02, 0.1));
  tfTree.getTF("eyeRight").setRPY(0.0, 0.0, 0.21);
  tfTree.getTF("shoulderLeft").setTrans(eeros::math::Vector3(0.2, 0.0, 0.5));
  tfTree.getTF("shoulderRight").setTrans(eeros::math::Vector3(-0.2, 0.0, 0.5));
  tfTree.getTF("ellbowRight").setTrans(eeros::math::Vector3(0.4, 0.0, 0.01));

  try {
    tfTree.getTF("shoulder").setTrans(eeros::math::Vector3(-0.2, 0.0, 0.5));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("TF_Tree: expected TF \"shoulder\"  could not be found"));
  }

//    TrafoMatrix* tf_eyeRight = tfTree.getTF("eyeRight");
}
