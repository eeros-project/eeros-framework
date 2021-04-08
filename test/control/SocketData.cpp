#include <eeros/control/SocketData.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlSocketDataTest, name) {
  SocketData<Vector2, Vector2> s("192.168.1.1", 9876);
  EXPECT_STREQ (s.getName().c_str(), "");

  s.setName("socket data block");
  EXPECT_STREQ (s.getName().c_str(), "socket data block");
}

// Test allocation
TEST(controlSocketDataTest, alloc) {
  SocketData<Vector2, int> s1("192.168.1.1", 9876);
  SocketData<Vector2, Matrix<2,3,double>> s2("192.168.1.1", 9876);
  SocketData<Vector2, Matrix<2,3,int>> s3("192.168.1.1", 9876);
  SocketData<int, Vector2> s4("192.168.1.1", 9876);
  SocketData<Matrix<2,3,double>, Vector2> s5("192.168.1.1", 9876);
  SocketData<Matrix<2,3,int>, Vector2> s6("192.168.1.1", 9876);
  SocketData<Matrix<2,3,bool>, bool> s7("192.168.1.1", 9876);
  SocketData<Vector2, std::nullptr_t> s8("192.168.1.1", 9876);
  SocketData<std::nullptr_t, Vector2> s9("192.168.1.1", 9876);
}
