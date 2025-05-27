#include <gtest/gtest.h>

#include <eeros/core/Fault.hpp>
#include <eeros/math/tf/TF_Tree.hpp>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::math::tf;

TEST(TF_TreeTest, init1) {
  TF_Tree& tfTree = TF_Tree::instance();
  tfTree.addTF("center", "global");
  tfTree.addTF("head", "center");
  tfTree.addTF("shoulderLeft", "center");
  tfTree.addTF("shoulderRight", "center");
  tfTree.addTF("eyeLeft", "head");
  tfTree.addTF("eyeRight", "head");
  tfTree.addTF("ellbowLeft", "shoulderLeft");
  tfTree.addTF("handLeft", "ellbowLeft");
  tfTree.addTF("ellbowRight", "shoulderRight");
  tfTree.addTF("handRight", "ellbowRight");

  try {
    tfTree.addTF("center", "head");
    FAIL();
  } catch (eeros::Fault const& err) {
    EXPECT_EQ(err.what(),
              std::string("TF_Tree: could not create node because a node with "
                          "name \"center\" already exists "));
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
  } catch (eeros::Fault const& err) {
    EXPECT_EQ(
        err.what(),
        std::string("TF_Tree: expected node \"shoulder\"  could not be found"));
  }

  TF_Matrix& eyeRight = tfTree.getTF("eyeRight");
  std::stringstream ss;
  ss << eyeRight.getMatrix();
  EXPECT_EQ(ss.str(),
            std::string("[ [0.978031 0.20846 0 0]' [-0.20846 0.978031 0 0]' [0 "
                        "0 1 0]' [0.06 -0.02 0.1 1]' ]"));

  TF_Matrix tf1 = tfTree.tfFrameToOrigin("head", "center");
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(),
            std::string("[ [0.987849 -0.129009 -0.086669 0]' [0.119114 "
                        "0.986656 -0.111007 0]' [0.0998334 0.0993347 0.990033 "
                        "0]' [-0.0698834 -0.0695343 -0.693023 1]' ]"));

  TF_Matrix tf2 = tfTree.tfFrameToOrigin("eyeLeft", "eyeRight");
  ss.str("");
  ss << tf2.getMatrix();
  EXPECT_EQ(ss.str(),
            std::string("[ [0.99995 0.00999983 0 0]' [-0.00999983 0.99995 0 "
                        "0]' [0 0 1 0]' [-0.00794677 -0.0392027 0 1]' ]"));

  TF_Matrix tf3 = tfTree.tfFrameToOrigin("ellbowRight", "shoulderRight");
  ss.str("");
  ss << tf3.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string("[ [1 0 0 0]' [0 1 0 0]' [0 0 1 0]' [-0.4 0 -0.01 1]' ]"));

  // adjoint representation from tf
  Matrix<6, 6, double> ad_ellbowRight_shoulderRight = tf3.getAdjointRep();
  ss.str("");
  ss << ad_ellbowRight_shoulderRight;
  EXPECT_EQ(
      ss.str(),
      std::string("[ [1 0 0 0 0 0]' [0 1 0 0 0 0]' [0 0 1 0 0 0]' [0 -0.01 -0 "
                  "1 0 0]' [0.01 0 -0.4 0 1 0]' [0 0.4 -0 0 0 1]' ]"));

  // adjoint representation from any matrix
  ad_ellbowRight_shoulderRight = TF_Matrix::getAdjointRep(tf3.getMatrix());
  ss.str("");
  ss << ad_ellbowRight_shoulderRight;
  EXPECT_EQ(
      ss.str(),
      std::string("[ [1 0 0 0 0 0]' [0 1 0 0 0 0]' [0 0 1 0 0 0]' [0 -0.01 -0 "
                  "1 0 0]' [0.01 0 -0.4 0 1 0]' [0 0.4 -0 0 0 1]' ]"));

  // transform velocity parameter, shoulder movements are measured
  Matrix<6, 1, double> velShoulder = {0.0, 0.0, 0.0, 0.0, 0.1, 0.3};
  ss.str("");
  ss << velShoulder;
  EXPECT_EQ(ss.str(), std::string("[0 0 0 0 0.1 0.3]' "));

  // with ad matrix the velocities for ellbow can be calculated:
  Matrix<6, 1, double> velEllbow = ad_ellbowRight_shoulderRight * velShoulder;
  ss.str("");
  ss << velEllbow;
  EXPECT_EQ(ss.str(), std::string("[0.001 0.12 -0.04 0 0.1 0.3]' "));

  // global positioning
  tfTree.getTF("center").setTrans(Vector3(10.0, 5.0, 0.3));
  tfTree.getTF("center").setRPY(0.0, 0.5, 1.7);
  // hand shoulder right in global view:
  TF_Matrix tf_shoulderRight_global =
      tfTree.tfFrameToOrigin("global", "shoulderRight");
  ss.str("");
  ss << tf_shoulderRight_global.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string(
          "[ [-0.113072 0.870268 -0.479426 0]' [-0.991665 -0.128844 0 0]' "
          "[-0.0617713 0.475429 0.877583 0]' [9.99173 5.06366 0.834676 1]' ]"));
}

TEST(TF_TreeTest, init2) {
  TF_Tree& tfTree = TF_Tree::instance();
  try {
    tfTree.addTF("center", "head");
    FAIL();
  } catch (eeros::Fault const& err) {
    EXPECT_EQ(err.what(),
              std::string("TF_Tree: could not create node because a node with "
              "name \"center\" already exists "));
  }
  TF_Matrix& eyeRight = tfTree.getTF("eyeRight");
  std::stringstream ss;
  ss << eyeRight.getMatrix();
  EXPECT_EQ(ss.str(),
            std::string("[ [0.978031 0.20846 0 0]' [-0.20846 0.978031 0 0]' [0 "
            "0 1 0]' [0.06 -0.02 0.1 1]' ]"));
}
