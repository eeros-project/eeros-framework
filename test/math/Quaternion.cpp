#include <eeros/math/Quaternion.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::math;

TEST(mathQuaternion, init) {
  Quaternion q1;
  EXPECT_TRUE(Utils::compareApprox(1, q1.get().norm(), 0.001));
  Quaternion q2 (2,5,3,4) ;
  EXPECT_TRUE(Utils::compareApprox(7.348, q2.get().norm(), 0.001));
  Matrix<4,1,double> v4{0.1, -0.5, 12, -245.5};
  Quaternion q3;
  q3.set(v4);
  EXPECT_TRUE(v4 == q3.get());
  q3.set({0.1, -0.5, 12, -245.5});
  EXPECT_TRUE(v4 == q3.get());
  v4.zero();
  v4[QUAT::x] = 3.0;
  v4[QUAT::z] = 5.0;
  v4[QUAT::w] = -7.0;
  v4[QUAT::y] = 0.3;
  q3.set(v4);
  EXPECT_EQ(q3.get()[0], -7.0);
  EXPECT_EQ(q3.get()[1], 3.0);
  EXPECT_EQ(q3.get()[2], 0.3);
  EXPECT_EQ(q3.get()[3], 5.0);
}

TEST(mathQuaternion, arith) {
  Matrix<4,1,double> v4{0.1, -0.5, 12, -245.5};
  Quaternion q1;
  q1.set(v4);
  q1 = q1 * 1;
  EXPECT_TRUE(v4 == q1.get());
  q1 *= 1;
  EXPECT_TRUE(v4 == q1.get());
  q1 *= 5;
  EXPECT_TRUE((v4 * 5) == q1.get());
  q1 = q1 / 5;
  EXPECT_TRUE(v4 == q1.get());
  q1 /= 2;
  EXPECT_TRUE(v4 / 2 == q1.get());
  q1.set(v4);
  Quaternion q2(2,5,3,4);
  Quaternion q3 = q1 + q2;
  EXPECT_TRUE(q1.get() + q2.get() == q3.get());
  q3 = q1;
  q3 += q2;
  EXPECT_TRUE(q1.get() + q2.get() == q3.get());
  Quaternion q4 = q1 - q2;
  EXPECT_TRUE(q1.get() - q2.get() == q4.get());
  q4 = q1;
  q4 -= q2;
  EXPECT_TRUE(q1.get() - q2.get() == q4.get());
  Quaternion q5 = q1 * q2;
  Matrix<4,1,double> m1{948.7, 784, -1201.2, -552.1};
  EXPECT_TRUE(q5.get() == m1);
  q5 = q1;
  q5 *= q2;
  EXPECT_TRUE(q5.get() == m1);
}

TEST(mathQuaternion, norm) {
  Quaternion q1(2,5,3,4);
  q1.normalize();
  Matrix<4,1,double> m1{0.27217, 0.68041, 0.40825, 0.54433};
  EXPECT_TRUE(Utils::compareApprox(q1.get()[0], m1[0], 0.001));
  EXPECT_TRUE(Utils::compareApprox(q1.get()[1], m1[1], 0.001));
  EXPECT_TRUE(Utils::compareApprox(q1.get()[2], m1[2], 0.001));
  EXPECT_TRUE(Utils::compareApprox(q1.get()[3], m1[3], 0.001));
}

TEST(mathQuaternion, conj) {
  Quaternion q1(2,5,3,4);
  Quaternion q2 = q1.conj();
  Matrix<4,1,double> m1{2, -5, -3, -4};
  EXPECT_EQ(q2.get(), m1);
}

TEST(mathQuaternion, inv) {
  Quaternion q1(2,5,3,4);
  Quaternion q2 = q1.inv();
  Quaternion q3 = q1 * q2;
  EXPECT_TRUE(Utils::compareApprox(q3.get().norm(), 1, 0.001));
  Quaternion q4(3,0,0,0);
  q2 = q1.inv();
  q3 = q1 * q2;
  EXPECT_TRUE(Utils::compareApprox(q3.get().norm(), 1, 0.001));
}

TEST(mathQuaternion, rot1) {
  // rotate 180deg around z
  Quaternion q1(0, 5.5 , 0 , 0);
  Quaternion q2(0, 0 , 0 , 1);
  Quaternion q3 = q2 * q1 * q2.inv();
  Matrix<4,1,double> m1{0, -5.5, 0, 0};
  EXPECT_EQ(q3.get(), m1);
  // rotate 90deg around z
  q1.set(0, 5.5, 5.5, 10);
  q2.set(0.707106781, 0, 0, 0.707106781);
  q3 = q2 * q1 * q2.inv();
  EXPECT_TRUE(Utils::compareApprox(q3.get()[0], 0, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[1], -5.5, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[2], 5.5, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[3], 10, 0.001));
  // rotate 180deg around x-z
  q1.set(0, 5.5, 5.5, 10);
  q2.set(0, -0.707106781 , 0,  0.707106781);
  q3 = q2 * q1 * q2.inv();
  EXPECT_TRUE(Utils::compareApprox(q3.get()[0], 0, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[1], -10, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[2], -5.5, 0.001));
  EXPECT_TRUE(Utils::compareApprox(q3.get()[3], -5.5, 0.001));
}

TEST(mathQuaternion, rot2) {
  Quaternion q1, q2;
  q1.setFromRPY(0, 0, 0.785398163);
  auto v1 = q1.getRPY();
  EXPECT_TRUE(Utils::compareApprox(v1[0], 0, 0.001));
  EXPECT_TRUE(Utils::compareApprox(v1[1], 0, 0.001));
  EXPECT_TRUE(Utils::compareApprox(v1[2], 0.785398163, 0.001));
  // calc rotation between two orientations
  // q1 * diff = q2  <=>  diff = q2 * inv(q1)
  q2.setFromRPY(0, 0, 3.14159);
  double turnZ = 3.14159 - 0.785398163;
  Quaternion  q3 = q2 * q1.inv();
  EXPECT_TRUE(Utils::compareApprox(2*std::acos(q3.get()[0]), turnZ, 0.001));
  q1.setFromRPY(0, 0, 3);
  q2.setFromRPY(0, 0, -3);
  turnZ = 2 * 3.14159 - 6;
  q3 = q2 * q1.inv();
  EXPECT_TRUE(Utils::compareApprox(2*std::acos(q3.get()[0]), turnZ, 0.001));
}
