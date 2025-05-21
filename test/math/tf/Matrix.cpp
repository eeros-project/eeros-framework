#include <gtest/gtest.h>

#include <eeros/core/Fault.hpp>
#include <eeros/math/tf/TF_Matrix.hpp>

using namespace eeros;
using namespace eeros::math;
using namespace eeros::math::tf;

TEST(TF_MatrixTest, roll) {
  TF_Matrix tf1;
  Matrix<4, 1, double> v1;
  Matrix<3, 1, double> v2;
  double r, p, y;
  r = M_PI / 3;
  p = 0.0;
  y = 0.0;
  tf1.setRPY(r, p, y);
  std::stringstream ss;
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [1 0 0 0]' [0 0.5 0.866025 0]' [0 "
                                  "-0.866025 0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[1.0472 0 0]' "));
  r = -M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [1 0 0 0]' [0 0.5 -0.866025 0]' [0 "
                                  "0.866025 0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[-1.0472 0 0]' "));
  r = 2 * M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [1 0 0 0]' [0 -0.5 0.866025 0]' [0 "
                                  "-0.866025 -0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[2.0944 0 0]' "));
  r = -2 * M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [1 0 0 0]' [0 -0.5 -0.866025 0]' [0 "
                                  "0.866025 -0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[-2.0944 0 0]' "));
}

TEST(TF_MatrixTest, pitch) {
  TF_Matrix tf1;
  Matrix<4, 1, double> v1;
  Matrix<3, 1, double> v2;
  double r, p, y;
  r = 0;
  p = M_PI / 3;
  y = 0;
  tf1.setRPY(r, p, y);
  std::stringstream ss;
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [0.5 0 -0.866025 0]' [0 1 0 0]' [0.866025 "
                                  "0 0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 1.0472 0]' "));
  p = -M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [0.5 0 0.866025 0]' [0 1 0 0]' [-0.866025 "
                                  "0 0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 -1.0472 0]' "));
}

TEST(TF_MatrixTest, yaw) {
  TF_Matrix tf1;
  Matrix<4, 1, double> v1;
  Matrix<3, 1, double> v2;
  double r, p, y;
  r = 0;
  p = 0.0;
  y = M_PI / 3;
  tf1.setRPY(r, p, y);
  std::stringstream ss;
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [0.5 0.866025 0 0]' [-0.866025 0.5 0 0]' "
                                  "[0 0 1 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 0 1.0472]' "));
  y = -M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [0.5 -0.866025 0 0]' [0.866025 0.5 0 0]' "
                                  "[0 0 1 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 0 -1.0472]' "));
  y = 2 * M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [-0.5 0.866025 0 0]' [-0.866025 -0.5 0 "
                                  "0]' [0 0 1 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 0 2.0944]' "));
  y = -2 * M_PI / 3;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(ss.str(), std::string("[ [-0.5 -0.866025 0 0]' [0.866025 -0.5 0 "
                                  "0]' [0 0 1 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0 0 -2.0944]' "));
}

TEST(TF_MatrixTest, mix) {
  TF_Matrix tf1;
  Matrix<4, 1, double> v1;
  Matrix<3, 1, double> v2;
  double r, p, y;
  r = M_PI / 4;
  p = M_PI / 3;
  y = M_PI / 2;
  tf1.setRPY(r, p, y);
  std::stringstream ss;
  ss << tf1.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string("[ [3.06162e-17 0.5 -0.866025 0]' [-0.707107 0.612372 "
                  "0.353553 0]' [0.707107 0.612372 0.353553 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[0.785398 1.0472 1.5708]' "));
  r = -M_PI / 4;
  p = -M_PI / 3;
  y = -M_PI / 5;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string(
          "[ [0.404508 -0.293893 0.866025 0]' [0.911047 0.212118 -0.353553 0]' "
          "[-0.0797928 0.932005 0.353553 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[-0.785398 -1.0472 -0.628319]' "));
  r = -M_PI / 4;
  p = -M_PI / 5;
  y = 5 * M_PI / 6;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string(
          "[ [-0.700629 0.404508 0.587785 0]' [-0.713497 -0.404559 -0.572061 "
          "0]' [0.0063901 -0.820186 0.572061 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[-0.785398 -0.628319 2.61799]' "));
  r = 3 * M_PI / 4;
  p = M_PI / 4;
  y = -5 * M_PI / 6;
  tf1.setRPY(r, p, y);
  ss.str("");
  ss << tf1.getMatrix();
  EXPECT_EQ(
      ss.str(),
      std::string("[ [-0.612372 -0.353553 -0.707107 0]' [-0.786566 0.362372 "
                  "0.5 0]' [0.0794593 0.862372 -0.5 0]' [0 0 0 1]' ]"));
  v2 = tf1.getRPY();
  ss.str("");
  ss << v2;
  EXPECT_EQ(ss.str(), std::string("[2.35619 0.785398 -2.61799]' "));
}

TEST(TF_MatrixTest, inv) {
  TF_Matrix tf1;
  tf1.setMatrix({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  Matrix<4, 4, double> m1 = tf1.inverse();
  Matrix<4, 4, double> m2;
  m2.eye();
  EXPECT_EQ(m1, m2);
  tf1.setMatrix(m2);
  m1 = tf1.inverse();
  EXPECT_EQ(m1, m2);
  tf1.setMatrix({2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
  m1 = tf1.inverse();
  m2 = {0.5, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  EXPECT_EQ(m1, m2);
  tf1.setMatrix({2, 0, 5, 0.25, 1, 1, 0, 0, 0, 0.5, 1, 0, 1, 0.5, 0, 1});
  m1 = tf1.inverse();
  std::stringstream ss;
  ss << m1;
  EXPECT_EQ(ss.str(),
            std::string("[ [0.228571 0.6 -1.14286 -0.0571429]' [-0.228571 0.4 "
                        "1.14286 0.0571429]' [0.114286 -0.2 0.428571 "
                        "-0.0285714]' [-0.114286 -0.8 0.571429 1.02857]' ]"));
}
