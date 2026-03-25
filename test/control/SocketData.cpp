#include <eeros/control/SocketData.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlSocketDataTest, name) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector2, Vector2> s("127.0.0.1", 9876, 0.1);
  EXPECT_STREQ (s.getName().c_str(), "");

  s.setName("socket data block");
  EXPECT_STREQ (s.getName().c_str(), "socket data block");
}

// Test allocation
TEST(controlSocketDataTest, alloc) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector2, int> s1("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, Matrix<2,3,double>> s2("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, Matrix<2,3,int>> s3("127.0.0.1", 9876, 0.1);
  SocketData<int, Vector2> s4("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,double>, Vector2> s5("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,int>, Vector2> s6("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,bool>, bool> s7("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, std::nullptr_t> s8("127.0.0.1", 9876, 0.1);
  SocketData<std::nullptr_t, Vector2> s9("127.0.0.1", 9876, 0.1);
  EXPECT_TRUE(true);
}
